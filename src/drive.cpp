#include "drive.h"
#include <algorithm>


namespace driving

{
	std::unordered_map<int, double> genDrivingLut() {
              std::unordered_map<int, double> umap;
              for(int i = -127; i <= 127; i++) {
                        umap[i] = expDrive(cvals(i));
              }
              return umap;
      	}

	Drive::Drive() : m_frontL{1}, m_backL{2}, m_frontR{3, true}, m_backR{4, true}
	{
	}
	
	void Drive::move(int leftX, int leftY, int rightX) {
		static const auto driveLut { genDrivingLut() };

		double cLeftX = driveLut.at(leftX);
		double cLeftY = driveLut.at(leftY);
		double cRightX = driveLut.at(rightX);

		double powerScale { 127.0 / std::max(std::abs(cLeftX) + std::abs(cLeftY) + std::abs(cRightX), 127.0)};
		double frontLWheel = (cLeftY + cLeftX - cRightX) * powerScale;
                double backLWheel = (cLeftY - cLeftX - cRightX) * powerScale;
                double frontRWheel = (cLeftY - cLeftX + cRightX) * powerScale;
                double backRWheel = (cLeftY + cLeftX + cRightX) * powerScale;
		
		constexpr double correctionFactor { 1.5625 }; //correction factor for 128 to 200 velocity control
		m_frontL.move_velocity(correctionFactor * frontLWheel);
                m_backL.move_velocity(correctionFactor * backLWheel);
                m_frontR.move_velocity(correctionFactor * frontRWheel);
                m_backR.move_velocity(correctionFactor * backRWheel);
		//std::cout << correctionFactor * frontLWheel << ", " << cLeftX << ", " << cLeftY << ", " << cRightX << ", " << powerScale << ", " << leftX << ", " << leftY << ", " << rightX << "\n";
	}
	
	void Drive::tiltLock(pros::Imu& imu, int tiltThreshold) {
                static bool tilted { false };
                int roll = std::abs(imu.get_roll());
                if (!tilted && roll >= 10 && roll < 1000) {
                        m_frontL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        m_backL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        m_frontR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        m_backR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        tilted = !tilted;
                }
                else if (tilted && roll < 10) {
                        m_frontL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        m_backL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        m_frontR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        m_backR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                        tilted = !tilted;
                }
		//std::cout << imu.get_roll() << "\n";
	
	}
	
	void Drive::shake(int magnitude, int power, int quantity) {
		for (int i=quantity; i > 0; i--){
			static bool forward { true };
			if (forward) {
				m_frontL.move(power);
				m_backL.move(power);
				m_frontR.move(power);
				m_backR.move(power);
				pros::delay(magnitude);
				forward = !forward;
				std::cout << "forward\n";
			}
			else {
				m_frontL.move(-power);
				m_backL.move(-power);
				m_frontR.move(-power);
				m_backR.move(-power);
				pros::delay(magnitude);
				forward = !forward;
				std::cout << "backward\n";
			}
		}

		m_frontL.move(0);
		m_backL.move(0);
		m_frontR.move(0);
		m_backR.move(0);
	
	}
}
