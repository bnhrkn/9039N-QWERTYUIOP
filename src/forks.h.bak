#ifndef FORKS_H
#define FORKS_H
#include <cstdint>
#include "main.h"

namespace forks {
	
class Fork {
	pros::Motor m_motor;
	std::uint8_t m_limit { 0 };
	void waitUntilMove(const pros::Motor& motor);
	void waitUntilStop(const pros::Motor& motor);
public:
	Fork(std::uint8_t port, bool reverse=false);
	void move(int speed);
};
}
#endif
