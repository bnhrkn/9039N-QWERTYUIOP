#include "main.h"
#include "drive.h"
#include "slide.h"


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

slide1::Slide slide (10, true);

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
struct inputValue {
    int left{ };
    int right{ };
};

void graph() {
    inputValue list[255]{ };
	for (int number{ 0 }; number < 255; ++number) {
		list[number].left = number - 127;
		std::cout << list[number].left << "," << driving::cvals(list[number].left) << "\n";
	}
}

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

//	graph();
	std::cout << frontR.is_stopped() << "\n";
	std::cout << frontR.get_actual_velocity() << "\n";
	std::cout << "before routine \n";
	slide.print();
	slide.calibrate();
	std::cout << "after routine \n";
	while (true) {
		// Get stick analog values, correct them, and set variables
		int leftStick { 0 };
		leftStick = driving::expDrive(driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)));
		int rightStick { 0 };
		rightStick = driving::expDrive(driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)));
		// Calculate the left and right speeds
		int leftPwr { 0 };
	    leftPwr = leftStick + rightStick;
		int rightPwr { 0 };
		rightPwr = leftStick - rightStick;
	    // Move the wheels using the calculated values
//		frontL.move(leftPwr);
//		backL.move(leftPwr);
//		frontR.move(rightPwr);
//	    backR.move(rightPwr);

//		std::cout << master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) << "," << master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//		std::cout << "," << driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) << "," << driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
//		std::cout << "," << driving::expDrive(driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X))) << "," << driving::expDrive(driving::cvals(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y))) << "\n";
		pros::delay(4);
	}
}
