#include "drive.h"
#include <cmath>
#include <array>

namespace driving
{
	int cvals(int analog) { //control values function, returns control value for inputs
	        if (std::abs(analog) < 110) {
	                return std::round(analog * 1.154545);
	        }
	        else {
	                return std::copysign(127, analog);
	        }
	}


//	int expDrive(int input) {
//		int absIn { std::abs(input) };
//		if (absIn <= 55) {
//			return input;
//	    	}
//		else {
//	        	return std::round(std::copysign( 0.0028 * absIn * absIn + 0.84545 * absIn, input ));
//	    	}
//	}
//	constexpr int expDrive(int input) {
//		constexpr double aCoe { 0.015749 };
//		constexpr double bCoe { -1.50012 };
//		constexpr double cCoe { 63.5 };
//		if (input <= 64 && input >= -64) {
//			return std::round(input/2.0);
//		}
//		else {
//			return std::round(copysign(aCoe * pow(input, 2), input) + bCoe * input + copysign(cCoe, input));
//		}
//	}
//
//	constexpr auto genDrivingLut() {
//		std::array<int, 255> arr {};
//
//		for(int i = 0; i < 255; i++) {
//			arr[i] = expDrive(i - 127);
//		}
//
//		return arr;
//	}
//

//
//	static_assert(drivingLut[100] == 113, "drivingLut: Incorrect output");
}
