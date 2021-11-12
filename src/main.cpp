#include "main.h"
#include "drive.h"
#include "slide.h"
#include <cmath>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
		
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
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
//class Slide {
//	int m_stop{ 0 }; //Number of degrees until the slide motor will refuse to extend
//
//public:
//	void calibrate() { //Determine the m_stop value
//		motor.move(-127)
//
//	}
//	void fullMove(bool position) { //Method to either fully extend or retract slide depending on a bool
//		if (position) {
//			
//		}
//	}
//
//}

slider::Slide slide (10, true);

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

void opcontrol() {
	// Construct the controller
    pros::Controller master (pros::E_CONTROLLER_MASTER);

	// Construct each of the 4 chassis movement motors
    pros::Motor frontL (1);
    pros::Motor frontR (3, true);
    pros::Motor backL (2);
    pros::Motor backR (4, true);
    // Construct the slide motor
    //pros::Motor slide (5);
	// Construct the lift motor
	pros::Motor lift (6);
    // Set brake mode for select motors
	//slide.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	int translateY { 0 };
	int translateX { 0 };
	int rotation { 0 };
	int frontLWheel { 0 };
	int backLWheel { 0 };
	int frontRWheel { 0 };
	int backRWheel { 0 };


	constexpr auto driveLut { driving::genDrivingLut() };

	for (int i = 0; i < 255; i++){
		std::cout << i << ", " << driveLut[i] <<"\n";
	}

	

	while (true) {
		// Get stick analog values, correct them, and set variables
//		leftStick = driving::expDrive(driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
//		rightStick = driving::expDrive(driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
		
		translateY = driveLut[127 + driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))];
		translateX = driveLut[127 + driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))];
		rotation = driveLut[127 - driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))];
		
		frontLWheel = translateY + translateX - rotation;
		backLWheel = translateY - translateX - rotation;
		frontRWheel = translateY - translateX + rotation;
		backRWheel = translateY + translateX + rotation;

		frontL.move(frontLWheel);
		backL.move(backLWheel);
		frontR.move(frontRWheel);
		backR.move(backRWheel);
		
//		std::cout << translateY << ", " << translateX << ", " << rotation << "\n";

//		std::cout << master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) << ", " << driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) << ", " << driveLut[127 + driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))] << "\n";
		// Calculate the left and right speeds
//		int leftPwr { 0 };
//	    leftPwr = leftStick + rightStick;
//		int rightPwr { 0 };
//		rightPwr = leftStick - rightStick;
	    // Move the wheels using the calculated values
//		frontL.move(leftPwr);
//		backL.move(leftPwr);
//		frontR.move(rightPwr);
//	    backR.move(rightPwr);
		
		lift.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		
//		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
//			slide.move(127);
//		}
//		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
//			slide.move(-127);
//		}
//		else {
//			slide.move(0);
//		}

		pros::delay(4);
	}
}
