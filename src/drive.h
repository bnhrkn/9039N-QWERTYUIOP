#ifndef DRIVE_H
#define DRIVE_H
#include <cmath>
#include <array>

namespace driving {
	int cvals(int analog);
//	constexpr int expDrive(int input);
        constexpr int expDrive(int input) {
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
	
	constexpr auto genDrivingLut() {
                std::array<int, 255> arr {};

                for(int i = 0; i < 255; i++) {
                        arr[i] = expDrive(i - 127);
                }

                return arr;
	}

	void drive();
}
#endif
