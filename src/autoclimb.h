#ifndef AUTOCLIMB_H
#define AUTOCLIMB_H
#include "main.h"
#include <cmath>

class wallSensor {
	std::shared_ptr<okapi::OdomChassisController>& chassis;
	auto lFront { okapilib::ADIUltrasonic(1, 2, std::make_unique<MedianFilter<5>>()) };
	auto lBack { okapilib::ADIUltrasonic(3, 4, std::make_unique<MedianFilter<5>>()) };
	auto rFront { okapilib::ADIUltrasonic(5, 6, std::make_unique<MedianFilter<5>>()) };
	auto rBack { okapilib::ADIUltrasonic(7, 8, std::make_unique<MedianFilter<5>>()) };
public:
	wallSensor();
	void align(bool left, const double wheelBase);











#endif
