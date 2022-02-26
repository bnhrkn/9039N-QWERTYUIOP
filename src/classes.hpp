#pragma once
#include "main.h"
#include <algorithm>
#include <cmath>

class cXDriveModel : public okapi::XDriveModel
{
    using okapi::XDriveModel::XDriveModel;

public:
    void driveVector(const double iforwardSpeed, const double iyaw);
    void xArcadeVel(const double ixSpeed,
                    const double iforwardSpeed,
                    const double iyaw,
                    const double ithreshold);
};

class cChassisControllerPID : public okapi::ChassisControllerPID
{
protected:
    const pros::Imu &Imu;

public:
    cChassisControllerPID(
        okapi::TimeUtil itimeUtil,
        std::shared_ptr<okapi::ChassisModel> ichassisModel,
        std::unique_ptr<okapi::IterativePosPIDController> idistanceController,
        std::unique_ptr<okapi::IterativePosPIDController> iturnController,
        std::unique_ptr<okapi::IterativePosPIDController> iangleController,
        const okapi::AbstractMotor::GearsetRatioPair &igearset,
        const okapi::ChassisScales &iscales,
        std::shared_ptr<okapi::Logger> ilogger,
        const pros::Imu &iIMU) // Add IMU to class
        : okapi::ChassisControllerPID(
              itimeUtil,
              ichassisModel,
              std::move(idistanceController),
              std::move(iturnController),
              std::move(iangleController),
              igearset,
              iscales,
              ilogger),
          Imu(iIMU)
    {
    }
    ~cChassisControllerPID()
    {
        dtorCalled.store(true, std::memory_order_release);
        delete task;
    }
    void loop();
    void startThread();
    static void trampoline(void *context);
};

class cRotationSensor : public okapi::RotationSensor
{
    using okapi::RotationSensor::RotationSensor;

public:
    double get() const;
};