#include "main.h"
#include <cmath>
#include <chrono>
#include <string>
#include "../include/okapi/impl/util/configurableTimeUtilFactory.hpp"
#include <map>
#include <limits>
#include "classes.hpp"

using namespace okapi::literals;

auto logger = std::make_shared<okapi::Logger>(
	okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
	"/ser/sout",										// Output to the PROS terminal
	okapi::Logger::LogLevel::debug						// Show errors and warnings
);

//Construct the IMU
pros::Imu inertial(4);

// Construct the rotation sensor
auto rotationSensor{std::make_shared<cRotationSensor>(1, true)};
auto baseRotarySensor{std::static_pointer_cast<okapi::RotarySensor>(rotationSensor)};

// Construct the lift motors
okapi::MotorGroup liftGroup({-3, 9});

double liftkP = 0.06;
double liftkI = 0.0005;
double liftkD = 0.0006;
std::shared_ptr<okapi::AsyncPositionController<double, double>> liftControl =
	okapi::AsyncPosControllerBuilder().withMotor(liftGroup).withSensor(baseRotarySensor).withGains({liftkP, liftkI, liftkD}).withGearset(okapi::AbstractMotor::GearsetRatioPair(okapi::AbstractMotor::gearset::red, 1)).withTimeUtilFactory(okapi::ConfigurableTimeUtilFactory(1, std::numeric_limits<double>::max(), 0_ms)).withLogger(logger).build();

// Drive Motors
auto topLeft{std::make_shared<okapi::Motor>(2)};
auto topRight{std::make_shared<okapi::Motor>(-7)};
auto bottomRight{std::make_shared<okapi::Motor>(-11)};
auto bottomLeft{std::make_shared<okapi::Motor>(20)};

// Drivemodel
auto XDriveTrain{std::make_shared<cXDriveModel>(
	topLeft,
	topRight,
	bottomRight,
	bottomLeft,
	topLeft->getEncoder(),
	topRight->getEncoder(),
	200,
	12000)};

// Chassis Controller PID Controllers
//auto chassisTimeUtilFactory = okapi::ConfigurableTimeUtilFactory(1, 0.5, 50_ms);
auto chassisTimeUtilFactory = okapi::TimeUtilFactory();

// Construct the Chassis Controller
auto drive{std::make_shared<cChassisControllerPID>(
	chassisTimeUtilFactory.create(),
	XDriveTrain,
	std::make_unique<okapi::IterativePosPIDController>(				   //Distance
		okapi::IterativePosPIDController::Gains{0.0019, 0.0, 0.00002}, // 0.0018 0.00018 0.0002
		chassisTimeUtilFactory.create()),
	std::make_unique<okapi::IterativePosPIDController>( //Turn
		okapi::IterativePosPIDController::Gains{0.0013, 0.00015, 0.0000},
		chassisTimeUtilFactory.create()),
	std::make_unique<okapi::IterativePosPIDController>( //Angle
		okapi::IterativePosPIDController::Gains{0.0013, 0.00015, 0.0000},
		chassisTimeUtilFactory.create()),
	okapi::AbstractMotor::gearset::green,
	okapi::ChassisScales({4_in, 15.5_in}, okapi::imev5GreenTPR),
	logger,
	inertial)};
/*
std::shared_ptr<okapi::ChassisController> drive = 
	okapi::ChassisControllerBuilder()
		.withMotors(
				2, 
				-7, 
				-11, 
				20
			   )
		.withDimensions(okapi::AbstractMotor::gearset::green, {{4_in, 15.5_in}, okapi::imev5GreenTPR})
		.build();
*/
auto cXDriveTrain{std::static_pointer_cast<cXDriveModel>(XDriveTrain)};
auto liftPIDController{std::dynamic_pointer_cast<okapi::AsyncPosPIDController>(liftControl)};

// Construct the controller
okapi::Controller controller;
okapi::ControllerButton slideButton(okapi::ControllerDigital::A);
okapi::ControllerButton holdButton(okapi::ControllerDigital::B);
okapi::ControllerButton forksLow(okapi::ControllerDigital::R2);
okapi::ControllerButton forksCarry(okapi::ControllerDigital::R1);
okapi::ControllerButton forksLoad(okapi::ControllerDigital::L2);
okapi::ControllerButton forksStore(okapi::ControllerDigital::L1);
okapi::ControllerButton eStop(okapi::ControllerDigital::X);
okapi::ControllerButton buttonY(okapi::ControllerDigital::Y);
okapi::ControllerButton buttonB(okapi::ControllerDigital::B);
okapi::ControllerButton buttonA(okapi::ControllerDigital::A);
okapi::ControllerButton buttonUp(okapi::ControllerDigital::up);
okapi::ControllerButton buttonDown(okapi::ControllerDigital::down);

// A callback function for LLEMU's center button.
void on_center_button()
{
}

int autonMode{3}; // Autonomous Mode Variable set by selector
#include "display.hpp"

void initialize()
{
	drive->startThread();
	display::startScreen();

	okapi::Logger::setDefaultLogger(
		std::make_shared<okapi::Logger>(
			okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
			"/ser/sout",										// Output to the PROS terminal
			okapi::Logger::LogLevel::info						// Show errors and warnings
			));
	liftGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

void disabled()
{
	liftControl->flipDisable(true);
	liftGroup.moveVelocity(0);
	cXDriveTrain->stop();
}

void competition_initialize()
{
}

void autonomous()
{
	switch (autonMode)
	{
	case 0: // Dual home goal wp
		liftControl->flipDisable(false);
		liftControl->setTarget(95);
		drive->turnAngle(0_deg);
		pros::delay(50);
		liftControl->waitUntilSettled();
		pros::delay(250);
		drive->moveDistance(13_in);
		liftControl->setTarget(55);
		pros::delay(400);
		drive->moveDistance(-8_in);
		cXDriveTrain->strafe(150);
		pros::delay(1000);
		cXDriveTrain->stop();
		drive->turnAngle(0_deg);
		liftControl->setTarget(95);
		pros::delay(250);
		drive->moveDistance(103_in);
		liftControl->setTarget(55);
		pros::delay(50);
		liftControl->waitUntilSettled();
		pros::delay(200);
		drive->moveDistance(-4_ft);
		//cXDriveTrain->driveVector(-150, 0);
		//pros::delay(1000);
		cXDriveTrain->driveVector(-100, 0);
		pros::delay(300);
		cXDriveTrain->driveVector(150, 0);
		pros::delay(200);
		cXDriveTrain->driveVector(-100, 0);
		pros::delay(300);
		cXDriveTrain->driveVector(150, 0);
		pros::delay(200);
		cXDriveTrain->stop();
		drive->turnAngle(-45_deg); /*
			liftControl->setTarget(95);
				pros::delay(50);
				liftControl->waitUntilSettled();
				pros::delay(200);
			drive->moveDistance(4_ft);
			liftControl->setTarget(55);
				pros::delay(50);
				liftControl->waitUntilSettled();
			drive->moveDistance(-3_ft);*/
		break;
	case 1: // Left Side Fast Neutral and score hg
		liftControl->flipDisable(false);
		drive->turnAngle(10_deg);
		liftControl->setTarget(95);
		//pros::delay(50);
		//liftControl->waitUntilSettled();
		pros::delay(650);
		drive->moveDistance(47_in);
		liftControl->setTarget(55);
		pros::delay(100);
		drive->turnAngle(28_deg);
		drive->moveDistance(-51_in);
		drive->turnAngle(93_deg);
		liftControl->setTarget(95);
		pros::delay(50);
		liftControl->waitUntilSettled();
		pros::delay(200);
		drive->moveDistance(17_in);
		liftControl->setTarget(55);
		pros::delay(500);
		//cXDriveTrain->driveVector(-150, 0);
		//pros::delay(500);
		//cXDriveTrain->stop();
		drive->moveDistance(-13_in);
		drive->turnAngle(45_deg);
		drive->moveDistance(3_ft);
		cXDriveTrain->driveVector(-100, 0);
		pros::delay(300);
		cXDriveTrain->driveVector(150, 0);
		pros::delay(200);
		cXDriveTrain->driveVector(-100, 0);
		pros::delay(300);
		cXDriveTrain->driveVector(150, 0);
		pros::delay(200);
		cXDriveTrain->stop();
		break;
	case 2: // Right Side Fast neutral and score hg
		liftControl->flipDisable(false);
		liftControl->setTarget(95);
		drive->turnAngle(0_deg);
		pros::delay(650);
		drive->moveDistance(45_in);
		liftControl->setTarget(55);
		pros::delay(500);
		drive->moveDistance(-45_in);
		liftControl->setTarget(95);
		drive->turnAngle(45_deg);
		liftControl->waitUntilSettled();
		drive->moveDistance(22_in);
		liftControl->setTarget(55);
		pros::delay(500);
		cXDriveTrain->driveVector(-150, 0);
		pros::delay(500);
		cXDriveTrain->stop();
		drive->turnAngle(0_deg);
		drive->moveDistance(2_ft);

		cXDriveTrain->driveVector(-100, 0);
		pros::delay(300);
		cXDriveTrain->driveVector(150, 0);
		pros::delay(200);
		cXDriveTrain->driveVector(-100, 0);
		pros::delay(300);
		cXDriveTrain->driveVector(150, 0);
		pros::delay(200);
		cXDriveTrain->stop();
		drive->turnAngle(-90_deg);
		//Continue moving further
		break;
		//default:
		//Do nothing
	}
}

void opcontrol()
{
	cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	liftControl->flipDisable(true);
	okapi::AverageFilter<32> leftXFilter;
	okapi::AverageFilter<32> leftYFilter;
	okapi::AverageFilter<32> rightXFilter;

	controller.rumble("-"); //Rumble the controller as a warning that operator control is starting

	while (true)
	{
		//Driving
		double drivingDeadzone{0.05};
		cXDriveTrain->xArcadeVel(
			controller.getAnalog(okapi::ControllerAnalog::leftX),
			controller.getAnalog(okapi::ControllerAnalog::leftY),
			controller.getAnalog(okapi::ControllerAnalog::rightX),
			drivingDeadzone);

		if (eStop.isPressed())
		{
			std::cout << "target: " << liftControl->getTarget() << " error: " << liftControl->getError() << " position: " << rotationSensor->get() << " settled: " << liftControl->isSettled() << "\n";
		}

		double rightY = controller.getAnalog(okapi::ControllerAnalog::rightY);

		// Manual fork control with right stick y-axis
		if (std::abs(rightY) > 0.05) // Deadzone
		{
			if (!liftControl->isDisabled()) // Dont spam flipdisable, fills log
			{
				liftControl->flipDisable(true); // Stop any automated movement
			}
			liftGroup.moveVelocity(rightY * 100); // Move forks according to stick
		}
		else if (liftControl->isDisabled()) // Stop forks during manual control below deadzone
		{
			liftGroup.moveVelocity(0);
		}

		// Fork Macros
		if (forksLow.changedToPressed()) // Forks on ground position
		{
			liftControl->flipDisable(false); // Enable controller after potential manual control disablement
			liftControl->setTarget(95);		 // Loading target
		}									 /*
		if (forksCarry.changedToPressed()) {
			liftControl->flipDisable(false);
			liftControl->setTarget(85);
		}*/
		if (forksLoad.changedToPressed())	 // Forks load goal position
		{
			liftControl->flipDisable(false);
			liftControl->setTarget(55);
		} /*
		if (forksStore.changedToPressed()) {
            liftControl->flipDisable(false);
            liftControl->setTarget(-5);
        }*/

		//Tilt Lock
		static bool tilted{false};				  // State variable holding tilt status
		int roll = std::abs(inertial.get_roll()); // Grab the IMU roll (up and down front tilt)
		if (!tilted && roll >= 4 && roll < 1000)  // Minimum roll of 4 deg. IMU reading is inf during initalization, cap at 1000
		{
			LOG_INFO_S("Tilt braking engaged");
			cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
			tilted = !tilted;
		}
		else if (tilted && roll < 4)
		{
			LOG_INFO_S("Tilt brake disengaged");
			cXDriveTrain->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			tilted = !tilted;
		}
		/*
		static double ckP { 0.0000 };
		static double ckI { 0.0000 };
		static double ckD { 0.0000 };

		// PID Tuning
		static int pidsel { 0 };
		if (buttonY.changedToPressed()) {
			pidsel = 0;
		}
		if (buttonB.changedToPressed()) {
			pidsel = 1;
		}
		if (buttonA.changedToPressed()) {
			pidsel = 2;
		}
		if (pidsel == 0) {
			if (buttonUp.changedToPressed()){
				ckP = ckP + 0.0001;
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
				controller.setText(0, 0, std::to_string( ckP ));
			}
			if (buttonDown.changedToPressed()){
				ckP = ckP - 0.0001;
				controller.setText(0, 0, std::to_string( ckP ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
		}

		if (pidsel == 1) {
			if (buttonUp.changedToPressed()){
				ckI = ckI + 0.00001;
				controller.setText(0, 0, std::to_string( ckI ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
			if (buttonDown.changedToPressed()){
				ckI = ckI - 0.00001;
				controller.setText(0, 0, std::to_string( ckI ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
		}

		if (pidsel == 2) {
			if (buttonUp.changedToPressed()){
				ckD = ckD + 0.00001;
				controller.setText(0, 0, std::to_string( ckD ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
			if (buttonDown.changedToPressed()){
				ckD = ckD - 0.00001;
				controller.setText(0, 0, std::to_string( ckD ));
				std::cout << "gains: " << ckP << ", " << ckI << ", " << ckD << "\n";
			}
		}
		*/
		/*
			drive->setGains(
					okapi::IterativePosPIDController::Gains{ckP,ckI,ckD}, 
					okapi::IterativePosPIDController::Gains{0.0013, 0.00015,0.00001}, 
					okapi::IterativePosPIDController::Gains{0.0013,0.00015,0.00001});
*/

		pros::delay(10);
	}
}
