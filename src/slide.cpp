#include "slide.h"

namespace slider {

void Slide::waitUntilMove(const pros::Motor& motor) {
    bool moving { false };
    while (!moving) {
        if (abs(motor.get_actual_velocity()) != 0) {
            moving = true;
        }
        pros::delay(5);
    }
}

void Slide::waitUntilStop(const pros::Motor& motor) {
    bool moving { true };
    while (moving) {
//        if (motor.is_stopped() == 1) {
		if (abs(motor.get_actual_velocity()) == 0) {
			moving = false;
		}
        pros::delay(5);
	}
}

int Slide::get_position() {
	return m_motor.get_position();
}

void Slide::calibrate() {
	m_motor.move(127);
    pros::delay(125);
	m_motor.move(0);
	waitUntilStop(m_motor);
	m_motor.move(-127);
    waitUntilMove(m_motor);
    waitUntilStop(m_motor);
	m_motor.move(0);
	pros::delay(250);
    m_motor.tare_position();
	m_motor.move(127);
	waitUntilMove(m_motor);
	waitUntilStop(m_motor);
	m_motor.move(0);
	pros::delay(250);
	m_limit = m_motor.get_position(); 
}

void Slide::fullMove(int speed) { //sign of speed will determine extension or contraction
	m_motor.move(speed);
	waitUntilStop(m_motor);
	m_motor.move(0);
}

void Slide::move(int speed) { //sign of speed also determines extension or contraction

	m_motor.move(speed);
}

Slide::Slide(std::uint8_t port, bool reverse) : m_motor{port, reverse}, m_limit{ 0 }
    {
		m_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	}

}

//class Slide {
//
//	pros::Motor m_motor;
//	std::uint8_t m_limit { 0 };
//	void waitUntilMove(const pros::Motor& motor) {
//		bool moving { false };
//		while (!moving) {
//			if (motor.get_actual_velocity() != 0) {
//				moving = true;
//			}
//			pros::delay(5);
//		}
//	}
//	void waitUntilStop(const pros::Motor& motor) {
//		bool moving { true };
//		while (moving) {
//			if (motor.get_actual_velocity() == 0) {
//				moving = false;
//			}
//			pros::delay(5);
//		}
//	}
//
//
//public:
//	
//	Slide(std::uint8_t port, bool reverse=false) : m_motor{port, reverse}, m_limit{ 0 }
//	{
//	}
//
//	void calibrate() {
//		m_motor.move(50);
//		pros::delay(250);
//		m_motor.move(-50);
//		waitUntilMove(m_motor);
//		waitUntilStop(m_motor);
//		m_motor.tare_position();
//	}
//};
//
//
