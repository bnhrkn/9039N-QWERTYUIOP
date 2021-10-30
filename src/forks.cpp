#include "forks.h"


namespace forks {

Fork::Fork(std::uint8_t port, bool reverse) : m_motor{port, pros::E_MOTOR_GEARSET_36, reverse}, m_limit{ 0} {
	m_motor.set_encoder_units(pros::E_MOTOR_ENCODER_COUNTS); 
}

void Fork::waitUntilMove(const pros::Motor& motor) {
	bool moving { false };
	while (!moving) {
        if (abs(motor.get_actual_velocity()) != 0) {
            moving = true;
        }
        pros::delay(5);
    }
}
void Fork::waitUntilStop(const pros::Motor& motor) {
	bool moving { true };
    while (moving) {
//        if (motor.is_stopped() == 1) {
        if (abs(motor.get_actual_velocity()) == 0) {
            moving = false;
        }
        pros::delay(5);
    }
}

void Fork::move(int speed) {
	m_motor.move(speed);
}
}
