#include "classes.hpp"

void cXDriveModel::driveVector(const double iforwardSpeed, const double iyaw)
{
    // This code is taken from WPIlib. All credit goes to them. Link:
    // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/Drive/DifferentialDrive.cpp#L73
    const double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
    const double yaw = std::clamp(iyaw, -1.0, 1.0);

    double leftOutput = forwardSpeed + yaw;
    double rightOutput = forwardSpeed - yaw;
    if (const double maxInputMag = std::max<double>(std::abs(leftOutput), std::abs(rightOutput));
        maxInputMag > 1)
    {
        leftOutput /= maxInputMag;
        rightOutput /= maxInputMag;
    }

    topLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
    topRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
    bottomRightMotor->moveVelocity(static_cast<int16_t>(rightOutput * maxVelocity));
    bottomLeftMotor->moveVelocity(static_cast<int16_t>(leftOutput * maxVelocity));
}

void cXDriveModel::xArcadeVel(const double ixSpeed,
                              const double iforwardSpeed,
                              const double iyaw,
                              const double ithreshold)
{
    double xSpeed = std::clamp(ixSpeed, -1.0, 1.0);
    if (std::abs(xSpeed) < ithreshold)
    {
        xSpeed = 0;
    }

    double forwardSpeed = std::clamp(iforwardSpeed, -1.0, 1.0);
    if (std::abs(forwardSpeed) < ithreshold)
    {
        forwardSpeed = 0;
    }

    double yaw = std::clamp(iyaw, -1.0, 1.0);
    if (std::abs(yaw) < ithreshold)
    {
        yaw = 0;
    }

    topLeftMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed + yaw, -1.0, 1.0) * maxVelocity));
    topRightMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed - yaw, -1.0, 1.0) * maxVelocity));
    bottomRightMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed + xSpeed - yaw, -1.0, 1.0) * maxVelocity));
    bottomLeftMotor->moveVelocity(
        static_cast<int16_t>(std::clamp(forwardSpeed - xSpeed + yaw, -1.0, 1.0) * maxVelocity));
}

double cRotationSensor::get() const
{
    const double out = pros::c::rotation_get_position(port);
    if (out == PROS_ERR_F)
    {
        return PROS_ERR_F;
    }
    else
    {
        // Convert from centidegrees to degrees and modulo to remove weird behavior
        return std::remainder(out * 0.01 * reversed, 360);
    }
}

void cChassisControllerPID::loop()
{
    LOG_INFO_S("Started ChassisControllerPID task.");

    auto encStartVals = chassisModel->getSensorVals();
    std::valarray<std::int32_t> encVals;
    double distanceElapsed = 0, angleChange = 0;
    modeType pastMode = none;
    auto rate = timeUtil.getRate();

    while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0))
    {
        /**
     * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
     * waitUntilSettled
     */
        if (doneLooping.load(std::memory_order_acquire))
        {
            doneLoopingSeen.store(true, std::memory_order_release);
        }
        else
        {
            if (mode != pastMode || newMovement.load(std::memory_order_acquire))
            {
                encStartVals = chassisModel->getSensorVals();
                newMovement.store(false, std::memory_order_release);
            }

            switch (mode)
            {
            case distance:
                encVals = chassisModel->getSensorVals() - encStartVals;
                distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
                angleChange = Imu.get_rotation() * scales.turn;
                distancePid->step(distanceElapsed);
                anglePid->step(angleChange - turnPid->getTarget());

                if (velocityMode)
                {
                    chassisModel->driveVector(distancePid->getOutput(), anglePid->getOutput());
                }
                else
                {
                    chassisModel->driveVectorVoltage(distancePid->getOutput(), anglePid->getOutput());
                }

                break;

            case angle:
                encVals = chassisModel->getSensorVals() - encStartVals;
                angleChange = Imu.get_rotation() * scales.turn;
                turnPid->step(angleChange);

                if (velocityMode)
                {
                    chassisModel->driveVector(0, turnPid->getOutput());
                }
                else
                {
                    chassisModel->driveVectorVoltage(0, turnPid->getOutput());
                }

                break;

            default:
                break;
            }

            pastMode = mode;
        }

        rate->delayUntil(threadSleepTime);
    }

    stop();

    LOG_INFO_S("Stopped ChassisControllerPID task.");
}

void cChassisControllerPID::startThread()
{
    if (!task)
    {
        task = new CrossplatformThread(trampoline, this, "ChassisControllerPID");
    }
}

void cChassisControllerPID::trampoline(void *context)
{
    if (context)
    {
        static_cast<cChassisControllerPID *>(context)->loop();
    }
}

void cChassisControllerPID::resetController()
{
    distancePid->reset();
    turnPid->reset();
    anglePid->reset();

    distancePid->flipDisable(true);
    turnPid->flipDisable(true);
    anglePid->flipDisable(true);

    distancePid->setTarget(0);
    turnPid->setTarget(0);
    anglePid->setTarget(0);
}
void cChassisControllerPID::moveDistance(const okapi::QLength itarget)
{
    moveDistanceAsync(itarget);
    pros::delay(40);
    waitUntilSettled();
}
void cChassisControllerPID::turnAngle(const okapi::QAngle idegTarget)
{
    turnAngleAsync(idegTarget);
    pros::delay(40);
    waitUntilSettled();
}