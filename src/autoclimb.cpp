#include "main.h"
#include <cmath>

constexpr double pi { 3.14159265358979323846 };
constexpr double radToDeg { 180 / pi };

namespace wall {


void wallSensor::align(bool left, const double wheelBase) {	
	double iLFront { lFront.get() };
	double iLBack { lBack.get() }:
	double iRFront { rFront.get() };
	double iRBack { rBack.get() };

	if (left {
		double dLOffset { ( iLfront + iRback ) / 2 };
		double dLFront { iLfront - dLOffset };
		double lAngle { std::atan( dLFront / wheelBase / 2 ) * radToDeg };
		chassis.turnRaw(lAngle);
	}
	else {
		double dROffset { ( iRFront + iRBack ) / 2 };
		double dRFront { iRFront - dROffset };
		double rAngle { std::atan( dRFront / wheelBase / 2 ) * radToDeg };
		chassis.turnRaw(rAngle);
	}


}
