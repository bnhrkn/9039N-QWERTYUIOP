#ifndef DRIVE_H
#define DRIVE_H
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include "main.h"

namespace driving {



template <typename val>
constexpr auto cvals(val analog) { //control values function, returns control value for inputs
	return std::clamp((analog * 1.154545), -127.0, 127.0);
}

template <typename val>
constexpr double expDrive(val input) {
        constexpr double aCoe { 0.015749 };
        constexpr double bCoe { -1.50012 };
        constexpr double cCoe { 63.5 };
        if (input <= 64 && input >= -64) {
                return input/2.0;
        }
        else {
                return copysign(aCoe * pow(input, 2), input) + bCoe * input + copysign(cCoe, input);
        }
}

std::unordered_map<int, double> genDrivingLut();

class Drive {
	pros::Motor m_frontL;
	pros::Motor m_backL;
	pros::Motor m_frontR;
	pros::Motor m_backR;
//	void jmove(int magnitude, int power);
public:
	Drive();
	void shake(int magnitude, int power, int quantity);
	void move(int leftX, int leftY, int rightX);
	void tiltLock(pros::Imu& imu, int tiltThreshold);
};


}
#endif
