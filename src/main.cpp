#include "main.h"
#include "drive.h"
#include "slide.h"
#include <cmath>

// A callback function for LLEMU's center button.
void on_center_button() {
}

// Runs initialization code. This occurs as soon as the program is started.
// All other competition modes are blocked by initialize
void initialize() {
		
}

// Runs while the robot is in the disabled state of Field Management System or
// the VEX Competition Switch, following either autonomous or opcontrol. When
// the robot is enabled, this task will exit.
void disabled() {}

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

slider::Slide slide (6, true);
driving::Drive drivetrain;

void autonomous() {
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
void shake (driving::Drive& drivetrain) {
	
}


void opcontrol() {
	// Construct the controller
    	pros::Controller master (pros::E_CONTROLLER_MASTER);

	// Construct each of the 4 chassis movement motors
	//pros::Motor frontL (1);
    	//pros::Motor frontR (3, true);
    	//pros::Motor backL (2);
    	//pros::Motor backR (4, true);
	// Construct the lift motor
	pros::Motor lift (5);
    	// Set brake mode for select motors
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	pros::Imu inertial(7);

	int translateY { 0 };
	int translateX { 0 };
	int rotation { 0 };
	int frontLWheel { 0 };
	int backLWheel { 0 };
	int frontRWheel { 0 };
	int backRWheel { 0 };


	auto driveLut { driving::genDrivingLut() };

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
		//Driving
		drivetrain.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
		
					master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
					-master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		drivetrain.tiltLock(inertial, 10);
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
		lift.move_velocity(1.5625 * driveLut[master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)]);
		
		//Slide
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			slide.fullMove(127);
		}
	
//		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
//                                drivetrain.shake(100, 127, 3);
//                                std::cout << "Shaking!!!\n";
//                }

		pros::delay(4);
	}
}
