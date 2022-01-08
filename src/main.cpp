#include "main.h"
//#include "drive.h"
//#include "slide.h"
#include <cmath>
#include <chrono>

using namespace okapi::literals;
pros::Imu inertial(7);
// Construct the rotation sensor
auto rotationSensor { std::make_shared<okapi::RotationSensor>(8, false) };
auto baseRotarySensor { std::dynamic_pointer_cast<okapi::RotarySensor>(rotationSensor) };
// Construct the lift motor
//okapi::Motor lift(5, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
std::shared_ptr<okapi::AsyncPositionController<double, double>> liftControl = 
	okapi::AsyncPosControllerBuilder().withMotor(5)
	.withSensor(baseRotarySensor)
	.withGearset(okapi::AbstractMotor::GearsetRatioPair(okapi::AbstractMotor::gearset::red, 8))
	.build();


// Construct the drivetrain
std::shared_ptr<okapi::OdomChassisController> drive = 
	okapi::ChassisControllerBuilder()
		.withMotors(
				1, 
				-3, 
				-4, 
				2
			   )
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 15.25_in}, okapi::imev5GreenTPR})
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
		.withOdometry()
		.buildOdometry();


std::shared_ptr<okapi::AsyncMotionProfileController> profiler = 
	okapi::AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 // Maximum linear jerk of the Chassis in m/s/s/s
			})
		.withOutput(drive)
		.buildMotionProfileController();


auto XDriveTrain { std::dynamic_pointer_cast<okapi::XDriveModel>(drive->getModel()) };
auto topLeft { XDriveTrain->getTopLeftMotor() };
// Construct the slide
slider::Slide slide (6, true);

// Construct the controller
okapi::Controller controller;
okapi::ControllerButton slideButton(okapi::ControllerDigital::A);
okapi::ControllerButton holdButton(okapi::ControllerDigital::B);
okapi::ControllerButton forksLow(okapi::ControllerDigital::R2);
okapi::ControllerButton forksCarry(okapi::ControllerDigital::R1);
okapi::ControllerButton forksLoad(okapi::ControllerDigital::L2);
okapi::ControllerButton eStop(okapi::ControllerDigital::X);
// okapi::ControllerButton forksStore();

// A callback function for LLEMU's center button.
void on_center_button() {
}

// Runs initialization code. This occurs as soon as the program is started.
// All other competition modes are blocked by initialize
void initialize() {
	profiler->generatePath({
			{0_ft, 0_ft, 0_deg},
			{1_ft, 1_ft, 0_deg}},
			"A"
			);
}

// Runs while the robot is in the disabled state of Field Management System or
// the VEX Competition Switch, following either autonomous or opcontrol. When
// the robot is enabled, this task will exit.
void disabled() {
	liftControl->controllerSet(0);
	slide.move(0);
	XDriveTrain->stop();
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
void competition_initialize() {}

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


void autonomous() {
//	profiler->setTarget("A");
//	profiler->waitUntilSettled();
	while(true) {
		auto start = pros::micros();
		auto accel = inertial.get_accel();
		auto stop = pros::micros();
//		std::cout << stop - start << std::endl;
		std::cout << accel.x << ", " << accel.y << ", " << accel.z << std::endl;
		pros::delay(1);
	}
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
//	constexpr double ialpha { 0.2 };
//	constexpr double ibeta {0.05 };

//	okapi::EmaFilter leftXFilter(ialpha, ibeta);
//	okapi::EmaFilter leftYFilter(ialpha, ibeta);
//	okapi::EmaFilter rightXFilter(ialpha, ibeta);
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


//	auto driveLut { driving::genDrivingLut() };

//	for (int i = -127; i <= 127; i++){
//		std::cout << i << ", " << driveLut[i] <<"\n";
//	}
//	int roll {0};
	
//	pros::Task shake{[&] {
//		while(true) {
//			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
//				drivetrain.shake(100, 127, 3);
//				std::cout << "Shaking!!!\n";
//			}
//			pros::delay(20);
//		}
//	}};
	while (true) {
//		auto start = pros::micros();
		//Driving
		double drivingDeadzone {0.05};
		double filteredRX {rightXFilter.filter(controller.getAnalog(okapi::ControllerAnalog::rightX))};
		double filteredLY {leftYFilter.filter(controller.getAnalog(okapi::ControllerAnalog::leftY))};
		double filteredLX {leftXFilter.filter(controller.getAnalog(okapi::ControllerAnalog::leftX))};
		XDriveTrain->xArcade( filteredLX, filteredLY, filteredRX, drivingDeadzone);
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
		liftControl->controllerSet(controller.getAnalog(okapi::ControllerAnalog::rightY)	);		
		
		if (forksLow.changedToPressed()) {
			liftControl->setTarget(85.5);
			std::cout << "liftControl pressed. Disabled: " << liftControl->isDisabled() << " Error: " << liftControl->getError() << "\n";
		}

		 

		//Slide
		if (slideButton.changedToPressed()) {
			slide.fullMove(127);
		}
		//Tilt Lock	
		static bool tilted { false };
		int roll = std::abs(inertial.get_roll());
		if (!tilted && roll >= 10 && roll < 1000) {
			XDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			tilted = !tilted;
		}
		else if (tilted && roll < 10) {
			XDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			tilted = !tilted;
		}
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

		switch (topLeft->getBrakeMode()) {
		case okapi::AbstractMotor::brakeMode::hold :
			std::cout << "hold mode\n";
			break;
		case okapi::AbstractMotor::brakeMode::coast :
			std::cout << "coast mode\n";
			break;
		}
//		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
//                                drivetrain.shake(100, 127, 3);
//                                std::cout << "Shaking!!!\n";
//                }
//		auto stop = pros::micros();
//		std::cout << stop - start  << std::endl;
		if (eStop.isPressed()) {
			liftControl->controllerSet(0);
		};
		pros::delay(10);
	}
}
