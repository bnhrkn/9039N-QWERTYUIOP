#include "main.h"
//#include "drive.h"
//#include "slide.h"
#include <cmath>
#include <chrono>
#include <string>

using namespace okapi::literals;

class cXDriveModel : public okapi::XDriveModel {
//  using okapi::XDriveModel::XDriveModel;
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






pros::Imu inertial(4);
// Construct the rotation sensor
auto rotationSensor { std::make_shared<cRotationSensor>(1, false) };
//auto crotationSensor { std::static_pointer_cast<cRotationSensor>(rotationSensor) };
//auto rotationSensor { std::make_shared<okapi::RotationSensor>(1, false) };
auto baseRotarySensor { std::static_pointer_cast<okapi::RotarySensor>(rotationSensor) };
// Construct the lift motors
//okapi::Motor lift(5, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
//auto liftGroup { std::make_shared<okapi::MotorGroup>(-2, 9)};
okapi::MotorGroup liftGroup({-3, 9});
//liftGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
//liftGroup.moveRelative(50);
double liftkP = 0.06;
double liftkI = 0.0005;
double liftkD = 0.0006;
std::shared_ptr<okapi::AsyncPositionController<double, double>> liftControl = 
	okapi::AsyncPosControllerBuilder().withMotor(liftGroup)
	.withSensor(baseRotarySensor)
	.withGains({liftkP, liftkI, liftkD})
	.withGearset(okapi::AbstractMotor::GearsetRatioPair(okapi::AbstractMotor::gearset::red, 1))
	.build();


// Construct the drivetrain

std::shared_ptr<okapi::ChassisController> drive = 
	okapi::ChassisControllerBuilder()
		.withMotors(
				2, 
				-10, 
				-11, 
				20
			   )
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 15.5_in}, okapi::imev5GreenTPR})
//		.withGains(
//			{0.00075, 0.0000, 0.0000}, //Distance controller gains
//			{0.000, 0, 0.000}, // Turn controller gain
//			{0.000, 0, 0.000} // Angle controller gains
//			)
//		.withDerivativeFilter(
//			std::make_unique<AverageFilter<3>>(), // Distance controller filter
//			std::make_unique<AverageFilter<3>>(), // Turn controller filter
//			std::make_unique<AverageFilter<3>>()  // Angle controller filter
//			)
//		.withOdometry()
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
// Runs initialization code. This occurs as soon as the program is started.
// All other competition modes are blocked by initialize
void initialize() {
	okapi::Logger::setDefaultLogger (
		std::make_shared<okapi::Logger> (
			okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
			"/ser/sout", // Output to the PROS terminal
			okapi::Logger::LogLevel::warn // Show errors and warnings
		)
	);
	liftGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

}

// Runs while the robot is in the disabled state of Field Management System or
// the VEX Competition Switch, following either autonomous or opcontrol. When
// the robot is enabled, this task will exit.
void disabled() {
	liftControl->flipDisable(true);
	liftGroup.moveVoltage(0);
//	slide.move(0);
	cXDriveTrain->stop();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {



}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void turnAngle() {
//	XDriveTrain->
}

void autonomous() {

//	liftControl->setTarget(95);
//	pros::delay(500);
//	drive->moveDistance(3_ft);
//	liftControl->setTarget(55);
//	pros::delay(500);
//	drive->moveDistance(-2_ft);
	liftControl->flipDisable(false);
	liftControl->setTarget(95);
	pros::delay(750);
	cXDriveTrain->forward(200);
	pros::delay(1000);
	cXDriveTrain->stop();
	pros::delay(250);
	liftControl->setTarget(55);
	pros::delay(500);
	cXDriveTrain->forward(-150);
	cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	pros::delay(750);
	cXDriveTrain->stop();
	liftControl->setTarget(0);
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
  provide “read-only” access to data. Therefore, the best practice is that they should retu* task, not resume it from where it left off.
 */


void opcontrol() {
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
		double filteredRX {rightXFilter.filter(controller.getAnalog(okapi::ControllerAnalog::rightX))};
		double filteredLY {leftYFilter.filter(controller.getAnalog(okapi::ControllerAnalog::leftY))};
		double filteredLX {leftXFilter.filter(controller.getAnalog(okapi::ControllerAnalog::leftX))};
		//XDriveTrain->xArcade( filteredLX, filteredLY, filteredRX, drivingDeadzone);
		cXDriveTrain->xArcadeVel( 
				controller.getAnalog(okapi::ControllerAnalog::leftX),
				controller.getAnalog(okapi::ControllerAnalog::leftY),
				controller.getAnalog(okapi::ControllerAnalog::rightX), 
				drivingDeadzone);
//		double drivingDeadzone {0.05};
//		XDriveTrain->xArcade(controller.getAnalog(okapi::ControllerAnalog::leftX),
//						controller.getAnalog(okapi::ControllerAnalog::leftY),
//						controller.getAnalog(okapi::ControllerAnalog::rightX),
//						drivingDeadzone);

		//
//		translateY = driveLut[master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)];
//		translateX = driveLut[master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)];
//		rotation = driveLut[-master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)];
//
//		frontLWheel = translateY + translateX - rotation;
//		backLWheel = translateY - translateX - rotation;
//		frontRWheel = translateY - translateX + rotation;
//		backRWheel = translateY + translateX + rotation;
//		
//		frontL.move_velocity(1.5625 * frontLWheel);
//		backL.move_velocity(1.5625 * backLWheel);
//		frontR.move_velocity(1.5625 * frontRWheel);
//		backR.move_velocity(1.5625 * backRWheel);
		//Forks
//		lift.move_velocity(1.5625 * driveLut[master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)]);
//		if (eStop.changedToPressed()) {
//
//		std::cout << 
//			"target: " << liftControl->getTarget() << 
//			" error: " << liftControl->getError() <<
//			" position: " << rotationSensor->get() <<
//			//" cposition: " << crotationSensor->get() <<
//			"\n";
//			while(eStop.isPressed()) {
//				liftGroup.moveVoltage(controller.getAnalog(okapi::ControllerAnalog::rightY)	);
//				pros::delay(5);
//			}
//		}
//		static double prevRightY;
		double rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);

		if (std::abs(rightY) > 0.05) {
			liftControl->flipDisable(true);
			liftGroup.moveVoltage(rightY * 12000);
		}
		else if (liftControl->isDisabled()) {
			liftGroup.moveVoltage(0);
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

//		switch (topLeft->getBrakeMode()) {
//		case okapi::AbstractMotor::brakeMode::hold :
//			std::cout << "hold mode\n";
//			break;
//		case okapi::AbstractMotor::brakeMode::coast :
//			std::cout << "coast mode\n";
//			break;
//		}
		pros::delay(10);
	}
}
