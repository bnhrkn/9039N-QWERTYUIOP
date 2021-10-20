#ifndef SLIDE_H
#define SLIDE_H
#include <stdint.h>
#include "main.h"

namespace slide1 {

class Slide {

	pros::Motor m_motor;
	std::uint8_t m_limit { 0 };
	void waitUntilMove(const pros::Motor& motor);
	void waitUntilStop(pros::Motor& motor);

public:
	 Slide(std::uint8_t port, bool reverse=false);
	 void print();
	 void calibrate();
};
}
#endif
