#ifndef SLIDE_H
#define SLIDE_H
#include <cstdint>
#include "main.h"

namespace slider {

class Slide {
	pros::Motor m_motor;
	std::uint8_t m_limit { 0 };
	void waitUntilMove(const pros::Motor& motor);
	void waitUntilStop(const pros::Motor& motor);
public:
	Slide(std::uint8_t port, bool reverse=false);
	void calibrate();
	void move(int speed);
	void fullMove(int speed);
	int get_position();
};
}
#endif
