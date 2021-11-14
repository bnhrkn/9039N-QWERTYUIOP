#ifndef DRIVE_H
#define DRIVE_H
#include <cmath>
#include <algorithm>
#include <unordered_map>

namespace driving {



template <typename val>
constexpr auto cvals(val analog) { //control values function, returns control value for inputs
	return std::clamp((analog * 1.154545), -127.0, 127.0);
}

template <typename val>
constexpr int expDrive(val input) {
        constexpr double aCoe { 0.015749 };
        constexpr double bCoe { -1.50012 };
        constexpr double cCoe { 63.5 };
        if (input <= 64 && input >= -64) {
                return std::round(input/2.0);
        }
        else {
                return std::round(copysign(aCoe * pow(input, 2), input) + bCoe * input + copysign(cCoe, input));
        }
}

std::unordered_map<int, int> genDrivingLut();

void drive();



}
#endif
