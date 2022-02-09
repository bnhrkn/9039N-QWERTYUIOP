#include "main.h"
#include <cmath>
#include <chrono>
#include <string>
#include "../include/okapi/impl/util/configurableTimeUtilFactory.hpp"

using namespace okapi::literals;

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

//Construct the IMU
pros::Imu inertial(4);

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
	.withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory(1, 0.5, 50_ms))
	.build();

// Construct the drivetrain
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

std::shared_ptr<okapi::AsyncMotionProfileController> profiler = 
	okapi::AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 // Maximum linear jerk of the Chassis in m/s/s/s
			})
		.withOutput(drive)
		.buildMotionProfileController();

//auto XDriveTrain { std::static_pointer_cast<okapi::XDriveModel>(drive->getModel()) };
auto cXDriveTrain { std::static_pointer_cast<cXDriveModel>(drive->getModel()) };
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
 
#include "display.hpp"

void initialize() {
	
	display::startScreen();	

	okapi::Logger::setDefaultLogger (
		std::make_shared<okapi::Logger> (
			okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
			"/ser/sout", // Output to the PROS terminal
			okapi::Logger::LogLevel::debug // Show errors and warnings
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
	liftControl->flipDisable(false);
	liftControl->setTarget(95);
	liftControl->waitUntilSettled();
	controller.rumble("-");
	/*
//	liftControl->setTarget(95);
//	pros::delay(500);
//	drive->moveDistance(3_ft);
//	liftControl->setTarget(55);
//	pros::delay(500);
//	drive->moveDistance(-2_ft);
	//cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	liftControl->flipDisable(false);
	liftControl->setTarget(95);
	pros::delay(750);
	cXDriveTrain->forward(200);
	pros::delay(1100);
	cXDriveTrain->stop();
	pros::delay(250);
	liftControl->setTarget(55);
	pros::delay(500);
	
	//strafe start

	pros::delay(250);

	cXDriveTrain->strafe(-150);
	pros::delay(1000);
	cXDriveTrain->stop();
	pros::delay(250);

	//strafe end

	cXDriveTrain->forward(-150);
	cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	pros::delay(1400); // straight reverse delay
//	pros::delay(800); // strafe reverse delay
	cXDriveTrain->stop();
//strafe turn
	cXDriveTrain->rotate(-200);
	pros::delay(350);
	cXDriveTrain->stop();
// strafe end turn
	liftControl->setTarget(0);
*/	
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
		if (eStop.changedToPressed()) {

		std::cout << 
			"target: " << liftControl->getTarget() << 
			" error: " << liftControl->getError() <<
			" position: " << rotationSensor->get() <<
			//" cposition: " << crotationSensor->get() <<
			"\n";
		/*	while(eStop.isPressed()) {
				liftGroup.moveVoltage(controller.getAnalog(okapi::ControllerAnalog::rightY)	);
				pros::delay(5);
			}*/
		}
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
		if (!tilted && roll >= 10 && roll < 1000) {
			std::cout << "triggered tiltbrake\n";
			cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			tilted = !tilted;
		}
		else if (tilted && roll < 10) {
			cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			tilted = !tilted;
		}


		// PID Tuning
//		static int pidsel { 0 };
//		if (buttonY.changedToPressed()) {
//			pidsel = 0;
//		}
//		if (buttonB.changedToPressed()) {
//			pidsel = 1;
//		}
//		if (buttonA.changedToPressed()) {
//			pidsel = 2;
//		}
//		if (pidsel == 0) {
//			if (buttonUp.changedToPressed()){
//				liftkP = liftkP + 0.001;
//				liftPIDController->setGains({liftkP, liftkI, liftkD});
//				std::cout << "gains: " << liftkP << ", " << liftkI << ", " << liftkD << "\n";
//				controller.setText(0, 0, std::to_string( liftkP ));
//			}
//			if (buttonDown.changedToPressed()){
//				liftkP = liftkP - 0.001;
//				controller.setText(0, 0, std::to_string( liftkP ));
//				std::cout << "gains: " << liftkP << ", " << liftkI << ", " << liftkD << "\n";
//				liftPIDController->setGains({liftkP, liftkI, liftkD});
//			}
//		}
//
//		if (pidsel == 1) {
//			if (buttonUp.changedToPressed()){
//				liftkI = liftkI + 0.0001;
//				controller.setText(0, 0, std::to_string( liftkI ));
//				std::cout << "gains: " << liftkP << ", " << liftkI << ", " << liftkD << "\n";
//				liftPIDController->setGains({liftkP, liftkI, liftkD});
//			}
//			if (buttonDown.changedToPressed()){
//				liftkI = liftkI - 0.0001;
//				controller.setText(0, 0, std::to_string( liftkI ));
//				std::cout << "gains: " << liftkP << ", " << liftkI << ", " << liftkD << "\n";
//				liftPIDController->setGains({liftkP, liftkI, liftkD});
//			}
//		}
//
//		if (pidsel == 2) {
//			if (buttonUp.changedToPressed()){
//				liftkD = liftkD + 0.0001;
//				controller.setText(0, 0, std::to_string( liftkD ));
//				std::cout << "gains: " << liftkP << ", " << liftkI << ", " << liftkD << "\n";
//				liftPIDController->setGains({liftkP, liftkI, liftkD});
//			}
//			if (buttonDown.changedToPressed()){
//				liftkD = liftkD - 0.0001;
//				controller.setText(0, 0, std::to_string( liftkD ));
//				std::cout << "gains: " << liftkP << ", " << liftkI << ", " << liftkD << "\n";
//				liftPIDController->setGains({liftkP, liftkI, liftkD});
//			}
//		}

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
