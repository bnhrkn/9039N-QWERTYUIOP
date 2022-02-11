#include "main.h"
#include <cmath>
#include <chrono>
#include <string>
#include "../include/okapi/impl/util/configurableTimeUtilFactory.hpp"
#include <map>
#include <limits>

using namespace okapi::literals;

auto logger = std::make_shared<okapi::Logger> (
            okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
            "/ser/sout", // Output to the PROS terminal
            okapi::Logger::LogLevel::debug // Show errors and warnings
        );

//Construct the IMU
pros::Imu inertial(4);

class cXDriveModel : public okapi::XDriveModel {
  public:
    void xArcadeVel(const double ixSpeed,
                          const double iforwardSpeed,
                          const double iyaw,
                          const double ithreshold) {
      double xSpeed = std::clamp(ixSpeed, -1.0, 1.0);
      if (std::abs(xSpeed) < ithreshold) {
        xSpeed = 0;
      }

      double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
      if (std::abs(forwardSpeed) < ithreshold) {
        forwardSpeed = 0;
      }

      double yaw = std::clamp(iyaw, -1.0, 1.0);
      if (std::abs(yaw) < ithreshold) {
        yaw = 0;
      }

      topLeftMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed + yaw, -1.0, 1.0) * maxVelocity));
      topRightMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed - yaw, -1.0, 1.0) * maxVelocity));
      bottomRightMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed - yaw, -1.0, 1.0) * maxVelocity));
      bottomLeftMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed + yaw, -1.0, 1.0) * maxVelocity));
    } 
};

class cRotationSensor : public okapi::RotationSensor {
	using okapi::RotationSensor::RotationSensor;
	public:
		double get() const {
			const double out = pros::c::rotation_get_position(port);
			if (out == PROS_ERR_F) {
				return PROS_ERR_F;
			} 
			else {
			// Convert from centidegrees to degrees and modulo to remove weird behavior
				return std::remainder(out * 0.01 * reversed, 360);
			}
		}
};
/*
class cIterativePosPIDController : public okapi::IterativePosPIDController {
	using okapi::IterativePosPIDController::IterativePosPIDController;
	public:
		double step(const double inewReading) {
		  if (controllerIsDisabled) {
			return 0;
		  } else {
			loopDtTimer->placeHardMark();

			if (loopDtTimer->getDtFromHardMark() >= sampleTime) {
			  // lastReading must only be updated here so its updates are time-gated by sampleTime
			  const double readingDiff = inertial.get_heading() - lastReading;
			  lastReading = inertial.get_heading();
				std::cout << "Reading: " << readingDiff;
			  error = getError();
				std::cout << "Error: " << error;
			  if ((std::abs(error) < target - errorSumMin && std::abs(error) > target - errorSumMax) ||
				  (std::abs(error) > target + errorSumMin && std::abs(error) < target + errorSumMax)) {
				integral += kI * error; // Eliminate integral kick while realtime tuning
			  }

			  if (shouldResetOnCross && std::copysign(1.0, error) != std::copysign(1.0, lastError)) {
				integral = 0;
			  }

			  integral = std::clamp(integral, integralMin, integralMax);

			  // Derivative over measurement to eliminate derivative kick on setpoint change
			  derivative = derivativeFilter->filter(readingDiff);

			  output = std::clamp(kP * error + integral - kD * derivative + kBias, outputMin, outputMax);

			  lastError = error;
			  loopDtTimer->clearHardMark(); // Important that we only clear if dt >= sampleTime

			  settledUtil->isSettled(error);
			}
		  }

		  return output;
	} 
};
*/
class cChassisControllerPID : public okapi::ChassisControllerPID {
protected:
	const pros::Imu& Imu;
public:
//	using okapi::ChassisControllerPID::ChassisControllerPID;
	cChassisControllerPID(
			okapi::TimeUtil itimeUtil,
			std::shared_ptr<okapi::ChassisModel> ichassisModel,
			std::unique_ptr<okapi::IterativePosPIDController> idistanceController,
			std::unique_ptr<okapi::IterativePosPIDController> iturnController,
			std::unique_ptr<okapi::IterativePosPIDController> iangleController,
			const okapi::AbstractMotor::GearsetRatioPair &igearset,
			const okapi::ChassisScales &iscales,
			std::shared_ptr<okapi::Logger> ilogger,
			const pros::Imu& iIMU)
			: okapi::ChassisControllerPID(
				itimeUtil,
				ichassisModel,
				std::move(idistanceController),
				std::move(iturnController),
				std::move(iangleController),
				igearset,
				iscales,
				ilogger
			),
			Imu(iIMU) {}
	~cChassisControllerPID() {
		dtorCalled.store(true, std::memory_order_release);
		delete task;
	}
	void loop() {
		LOG_INFO_S("Started ChassisControllerPID task.");

  auto encStartVals = chassisModel->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;
  modeType pastMode = none;
  auto rate = timeUtil.getRate();

  while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
    /**
     * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
     * waitUntilSettled
     */
    if (doneLooping.load(std::memory_order_acquire)) {
      doneLoopingSeen.store(true, std::memory_order_release);
    } else {
      if (mode != pastMode || newMovement.load(std::memory_order_acquire)) {
        encStartVals = chassisModel->getSensorVals();
        newMovement.store(false, std::memory_order_release);
      }

      switch (mode) {
      case distance:
        encVals = chassisModel->getSensorVals() - encStartVals;
        distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
        //angleChange = static_cast<double>(encVals[0] - encVals[1]);
		angleChange = Imu.get_rotation() * scales.turn;
        distancePid->step(distanceElapsed);
        anglePid->step(angleChange - turnPid->getTarget());

        if (velocityMode) {
          chassisModel->driveVector(distancePid->getOutput(), anglePid->getOutput());
        } else {
          chassisModel->driveVectorVoltage(distancePid->getOutput(), anglePid->getOutput());
        }

        break;

      case angle:
        encVals = chassisModel->getSensorVals() - encStartVals;
//        angleChange = (encVals[0] - encVals[1]) / 2.0;
		angleChange = Imu.get_rotation() * scales.turn;
		turnPid->step(angleChange);

        if (velocityMode) {
          chassisModel->driveVector(0, turnPid->getOutput());
        } else {
          chassisModel->driveVectorVoltage(0, turnPid->getOutput());
        }

        break;

      default:
        break;
      }

      pastMode = mode;
    }

    rate->delayUntil(threadSleepTime);
  }

  stop();

  LOG_INFO_S("Stopped ChassisControllerPID task.");
}
void startThread() {
	if (!task) {
    task = new CrossplatformThread(trampoline, this, "ChassisControllerPID");
  }
}
static void trampoline(void *context) {
  if (context) {
    static_cast<cChassisControllerPID *>(context)->loop();
  }
}


};



// Construct the rotation sensor
auto rotationSensor { std::make_shared<cRotationSensor>(1, false) };
auto baseRotarySensor { std::static_pointer_cast<okapi::RotarySensor>(rotationSensor) };

// Construct the lift motors
okapi::MotorGroup liftGroup({-3, 9});

double liftkP = 0.06;
double liftkI = 0.0005;
double liftkD = 0.0006;
std::shared_ptr<okapi::AsyncPositionController<double, double>> liftControl = 
	okapi::AsyncPosControllerBuilder().withMotor(liftGroup)
	.withSensor(baseRotarySensor)
	.withGains({liftkP, liftkI, liftkD})
	.withGearset(okapi::AbstractMotor::GearsetRatioPair(okapi::AbstractMotor::gearset::red, 1))
	.withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory(1, std::numeric_limits<double>::max(), 0_ms))
	.withLogger(logger)
	.build();

// Drive Motors
auto topLeft { std::make_shared<okapi::Motor>(2) };
auto topRight { std::make_shared<okapi::Motor>(-7) };
auto bottomRight { std::make_shared<okapi::Motor>(-11) };
auto bottomLeft { std::make_shared<okapi::Motor>(20) };

// Drivemodel
auto XDriveTrain { std::make_shared<okapi::XDriveModel>(
		topLeft,
		topRight,
		bottomRight,
		bottomLeft,
		topLeft->getEncoder(),
		topRight->getEncoder(),
		200,
		12000) };

// Chassis Controller PID Controllers
//auto chassisTimeUtilFactory = okapi::ConfigurableTimeUtilFactory(1, 0.5, 50_ms);
auto chassisTimeUtilFactory = okapi::TimeUtilFactory();

// Construct the Chassis Controller
auto drive { std::make_shared<cChassisControllerPID>(
		chassisTimeUtilFactory.create(),
		XDriveTrain,
		std::make_unique<okapi::IterativePosPIDController>( //Distance
            okapi::IterativePosPIDController::Gains{0.0019,0.0,0.00002}, // 0.0018 0.00018 0.0002
            chassisTimeUtilFactory.create()
        ),
		std::make_unique<okapi::IterativePosPIDController>( //Turn
            okapi::IterativePosPIDController::Gains{0.0013, 0.00015,0.0000},
            chassisTimeUtilFactory.create()
        ),
		std::make_unique<okapi::IterativePosPIDController>( //Angle
            okapi::IterativePosPIDController::Gains{0.0013,0.00015,0.0000},
            chassisTimeUtilFactory.create()
        ),
		okapi::AbstractMotor::gearset::green,
		okapi::ChassisScales({4_in,15.5_in}, okapi::imev5GreenTPR),
		logger,
		inertial
		)};
/*
std::shared_ptr<okapi::ChassisController> drive = 
	okapi::ChassisControllerBuilder()
		.withMotors(
				2, 
				-7, 
				-11, 
				20
			   )
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 15.5_in}, okapi::imev5GreenTPR})
		.build();
*/
/*
std::shared_ptr<okapi::AsyncMotionProfileController> profiler = 
	okapi::AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 // Maximum linear jerk of the Chassis in m/s/s/s
			})
		.withOutput(drive)
		.buildMotionProfileController();
*/
//auto XDriveTrain { std::static_pointer_cast<okapi::XDriveModel>(drive->getModel()) };
auto cXDriveTrain { std::static_pointer_cast<cXDriveModel>(XDriveTrain) };
//auto XDriveTrain { std::dynamic_pointer_cast<cXDriveModel>(drive->getModel()) };
//auto XDriveTrain { std::
auto liftPIDController { std::dynamic_pointer_cast<okapi::AsyncPosPIDController>(liftControl) };

// Construct the controller
okapi::Controller controller;
okapi::ControllerButton slideButton(okapi::ControllerDigital::A);
okapi::ControllerButton holdButton(okapi::ControllerDigital::B);
okapi::ControllerButton forksLow(okapi::ControllerDigital::R2);
okapi::ControllerButton forksCarry(okapi::ControllerDigital::R1);
okapi::ControllerButton forksLoad(okapi::ControllerDigital::L2);
okapi::ControllerButton forksStore(okapi::ControllerDigital::L1);
okapi::ControllerButton eStop(okapi::ControllerDigital::X);
okapi::ControllerButton buttonY(okapi::ControllerDigital::Y);
okapi::ControllerButton buttonB(okapi::ControllerDigital::B);
okapi::ControllerButton buttonA(okapi::ControllerDigital::A);
okapi::ControllerButton buttonUp(okapi::ControllerDigital::up);
okapi::ControllerButton buttonDown(okapi::ControllerDigital::down);

// A callback function for LLEMU's center button.
void on_center_button() {
}
/*
lv_obj_t* autonSelectorBtn;
lv_obj_t* autonSelectorBtnLbl;
lv_obj_t* autonSelectorScr;

lv_obj_t* testBtn;
lv_obj_t* testBtnLbl;

lv_obj_t* graphsBtn;
lv_obj_t* graphsBtnLbl;

lv_obj_t* calibrateBtn;
lv_obj_t* calibrateBtnLbl;
*/
 
int autonMode { 3 }; // Autonomous Mode Variable set by selector
#include "display.hpp"

void initialize() {
	drive->startThread();
	display::startScreen();	

	okapi::Logger::setDefaultLogger (
		std::make_shared<okapi::Logger> (
			okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
			"/ser/sout", // Output to the PROS terminal
			okapi::Logger::LogLevel::info // Show errors and warnings
		)
	);
	liftGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

}

void disabled() {
	liftControl->flipDisable(true);
	liftGroup.moveVelocity(0);
	cXDriveTrain->stop();
}

void competition_initialize() {
}


void autonomous() {
	switch(autonMode) {
		case 0: // Dual home goal wp
			liftControl->flipDisable(false);
			liftControl->setTarget(95);
				pros::delay(50);
				liftControl->waitUntilSettled();
			drive->moveDistance(15_in);
			liftControl->setTarget(55);
			cXDriveTrain->strafe(150);
			pros::delay(1350);
			cXDriveTrain->stop();
			drive->turnAngle(0_deg);
			liftControl->setTarget(95);
			pros::delay(250);
			drive->moveDistance(100_in);
			liftControl->setTarget(55);
				pros::delay(50);
				liftControl->waitUntilSettled();
			cXDriveTrain->forward(-200);
			pros::delay(1000);
			cXDriveTrain->stop();
		case 1: // Left Side Fast Neutral and score hg
			liftControl->flipDisable(false);
			liftControl->setTarget(95);
				//pros::delay(50);
				//liftControl->waitUntilSettled();
			drive->moveDistance(51_in);
			liftControl->setTarget(55);
			pros::delay(200);
			drive->moveDistance(-51_in);
			drive->turnAngle(90_deg);
			liftControl->setTarget(95);
				pros::delay(50);
				liftControl->waitUntilSettled();
			drive->moveDistance(15_in);
			liftControl->setTarget(55);
				pros::delay(50);
				liftControl->waitUntilSettled();
			cXDriveTrain->forward(-200);
			pros::delay(500);
			cXDriveTrain->stop();
			drive->turnAngle(45_deg);
			drive->moveDistance(2_ft);
			break;
		case 2: // Right Side Fast neutral and score hg
			break;
		//default:
			//Do nothing

	}
}

void opcontrol() {
	cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	liftControl->flipDisable(true);
	okapi::AverageFilter<32> leftXFilter;
	okapi::AverageFilter<32> leftYFilter;
	okapi::AverageFilter<32> rightXFilter;

	//Rumble the controller as a warning that operator control is starting
	controller.rumble("-");
	
	int translateY { 0 };
	int translateX { 0 };
	int rotation { 0 };
	int frontLWheel { 0 };
	int backLWheel { 0 };
	int frontRWheel { 0 };
	int backRWheel { 0 };


	while (true) {
		//Driving
		double drivingDeadzone {0.05};
		cXDriveTrain->xArcadeVel( 
				controller.getAnalog(okapi::ControllerAnalog::leftX),
				controller.getAnalog(okapi::ControllerAnalog::leftY),
				controller.getAnalog(okapi::ControllerAnalog::rightX), 
				drivingDeadzone);

		//Forks
//		lift.move_velocity(1.5625 * driveLut[master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)]);
		if (eStop.isPressed()) {
			drive->turnAngle(0_deg);
			drive->moveDistance(6_ft);
			//drive->turnAngle(90_deg);
			
//		std::cout << inertial.get_heading() << "\n";
/*
		std::cout << 
			"target: " << liftControl->getTarget() << 
			" error: " << liftControl->getError() <<
			" position: " << rotationSensor->get() <<
			" settled: " << liftControl->isSettled() <<
			//" cposition: " << crotationSensor->get() <<
			"\n";
			//while(eStop.isPressed()) {
			//	liftGroup.moveVoltage(controller.getAnalog(okapi::ControllerAnalog::rightY)	);
			//	pros::delay(5);
			//}*/
		} 

	//}
//		static double prevRightY;
		double rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);

		if (std::abs(rightY) > 0.05) {
			liftControl->flipDisable(true);
			liftGroup.moveVelocity(rightY * 100);
		}
		else if (liftControl->isDisabled()) {
			liftGroup.moveVelocity(0);
		}

// Rotary Sensor must be zeroed at precisely vertical arms.
		if (forksLow.changedToPressed()) {
			liftControl->flipDisable(false);
			liftControl->setTarget(95);
		}
		if (forksCarry.changedToPressed()) {
			liftControl->flipDisable(false);
			liftControl->setTarget(85);
		}
		if (forksLoad.changedToPressed()) {
			liftControl->flipDisable(false);
			liftControl->setTarget(55);
			//std::cout << "liftControl pressed. Disabled: " << liftControl->isDisabled() << " Error: " << liftControl->getError() << " Measure: " << rotationSensor->get() << "\n";
		}
		if (forksStore.changedToPressed()) {
            liftControl->flipDisable(false);
            liftControl->setTarget(-5);
        }
		//Tilt Lock	
		static bool tilted { false };
		int roll = std::abs(inertial.get_roll());
		if (!tilted && roll >= 3 && roll < 1000) {
			std::cout << "triggered tiltbrake\n";
			cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			tilted = !tilted;
		}
		else if (tilted && roll < 3) {
			cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			tilted = !tilted;
		}

		static double ckP { 0.0000 };
		static double ckI { 0.0000 };
		static double ckD { 0.0000 };

		// PID Tuning
		static int pidsel { 0 };
		if (buttonY.changedToPressed()) {
			pidsel = 0;
		}
		if (buttonB.changedToPressed()) {
			pidsel = 1;
		}
		if (buttonA.changedToPressed()) {
			pidsel = 2;
		}
		if (pidsel == 0) {
			if (buttonUp.changedToPressed()){
				ckP = ckP + 0.0001;
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
				controller.setText(0, 0, std::to_string( ckP ));
			}
			if (buttonDown.changedToPressed()){
				ckP = ckP - 0.0001;
				controller.setText(0, 0, std::to_string( ckP ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
		}

		if (pidsel == 1) {
			if (buttonUp.changedToPressed()){
				ckI = ckI + 0.00001;
				controller.setText(0, 0, std::to_string( ckI ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
			if (buttonDown.changedToPressed()){
				ckI = ckI - 0.00001;
				controller.setText(0, 0, std::to_string( ckI ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
		}

		if (pidsel == 2) {
			if (buttonUp.changedToPressed()){
				ckD = ckD + 0.00001;
				controller.setText(0, 0, std::to_string( ckD ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
			if (buttonDown.changedToPressed()){
				ckD = ckD - 0.00001;
				controller.setText(0, 0, std::to_string( ckD ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
		}

/*			drive->setGains(
					okapi::IterativePosPIDController::Gains{ckP,ckI,ckD}, 
					okapi::IterativePosPIDController::Gains{0.0013, 0.00015,0.00001}, 
					okapi::IterativePosPIDController::Gains{0.0013,0.00015,0.00001});
	*/				
//		static bool toggle { false };
//		if (holdButton.changedToPressed()) {
//			toggle = !toggle;
//			if (toggle) {
//				XDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
//			}
//			else {
//				XDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
//			}
//		}
		pros::delay(10);
	}
}
