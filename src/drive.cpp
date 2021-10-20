#include "drive.h"
#include <cmath>
#include <cstdint>


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
	
	int expDrive(int input) {
	//  return copysign(std::round(std::pow(abs(input), power) / std::pow(127, power - 1)), input);
	    if (std::abs(input) <= 55) {
	        return input;
	    }
		else {
	        return std::copysign(std::round(0.0028 * input * input + 0.84545 * input), input);
	    }
	}
	
//	void drive() {
//
//	}
}