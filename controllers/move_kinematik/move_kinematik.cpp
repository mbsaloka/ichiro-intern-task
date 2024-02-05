// File:          move_forward.cpp
// Date:          16 Jan 2024
// Description:   rectangle move on e-puck robots
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <chrono>
#include <iomanip>

#define TIME_STEP 16
#define MAX_SPEED 6.28
#define H 20
#define W 40
#define X MAX_SPEED
#define Y 60

using namespace webots;

int degreeIMU(InertialUnit *imu) {
    return ((int)(imu->getRollPitchYaw()[2] * (180 / M_PI)) % 360 + 360) % 360;
}

void getIMUDegrees(InertialUnit *imu) {
    int degrees =
        ((int)(imu->getRollPitchYaw()[2] * (180 / M_PI)) % 360 + 360) % 360;
    std::cout << degrees << "° degrees | " << std::fixed << std::setprecision(2)
              << -imu->getRollPitchYaw()[2] << " rad" << std::endl;
    // std::cout << "---------------------------------" << std::endl;
}

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    PositionSensor *leftPositionSensor =
        robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightPositionSensor =
        robot->getPositionSensor("right wheel sensor");
    leftPositionSensor->enable(TIME_STEP);
    rightPositionSensor->enable(TIME_STEP);

    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(TIME_STEP);

    robot->step(TIME_STEP);

    double pos;

    // 1
    pos = leftPositionSensor->getValue();
    while (robot->step(TIME_STEP) != -1 &&
           leftPositionSensor->getValue() < (pos + H / 2)) {
        leftMotor->setVelocity(X);
        rightMotor->setVelocity(X);
        getIMUDegrees(imu);
    };

    auto start_time = std::chrono::steady_clock::now();
    while (robot->step(TIME_STEP) != -1 &&
           imu->getRollPitchYaw()[2] > (-M_PI / 2) &&
           imu->getRollPitchYaw()[2] < (M_PI / 2)) {
        leftMotor->setVelocity(0.2 * X);
        rightMotor->setVelocity(0.2 * -X);
        getIMUDegrees(imu);
    }
    leftMotor->setVelocity(X);
    rightMotor->setVelocity(X);
    auto end_time = std::chrono::steady_clock::now();
    auto turn_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                         end_time - start_time)
                         .count() /
                     1000.0;

    // 2
    pos = leftPositionSensor->getValue();
    while (robot->step(TIME_STEP) != -1 &&
           leftPositionSensor->getValue() < (pos + W)) {
        leftMotor->setVelocity(X);
        rightMotor->setVelocity(X);
        getIMUDegrees(imu);
    }

    while (robot->step(TIME_STEP) != -1 && degreeIMU(imu) > 180) {
        leftMotor->setVelocity(0.2 * X);
        rightMotor->setVelocity(0.2 * -X);
        getIMUDegrees(imu);
    }

    // 3
    pos = leftPositionSensor->getValue();
    while (robot->step(TIME_STEP) != -1 &&
           leftPositionSensor->getValue() < (pos + H)) {
        leftMotor->setVelocity(X);
        rightMotor->setVelocity(X);
        getIMUDegrees(imu);
    }

    while (robot->step(TIME_STEP) != -1 && degreeIMU(imu) > 90) {
        leftMotor->setVelocity(0.2 * X);
        rightMotor->setVelocity(0.2 * -X);
        getIMUDegrees(imu);
    }

    // 4
    pos = leftPositionSensor->getValue();
    while (robot->step(TIME_STEP) != -1 &&
           leftPositionSensor->getValue() < (pos + W)) {
        leftMotor->setVelocity(X);
        rightMotor->setVelocity(X);
        getIMUDegrees(imu);
    }

    while (robot->step(TIME_STEP) != -1 && degreeIMU(imu) > 0 &&
           degreeIMU(imu) <= 90) {
        leftMotor->setVelocity(0.2 * X);
        rightMotor->setVelocity(0.2 * -X);
        getIMUDegrees(imu);
    }

    // 5
    pos = leftPositionSensor->getValue();
    while (robot->step(TIME_STEP) != -1 &&
           leftPositionSensor->getValue() < (pos + H / 2)) {
        leftMotor->setVelocity(X);
        rightMotor->setVelocity(X);
        getIMUDegrees(imu);
    }

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    double travelDistance = 2 * H + 2 * W;
    double travelTime = travelDistance / X + turn_time * 4;

    std::cout << "distance : " << travelDistance << " inches" << std::endl;
    std::cout << "speed : " << X << " inches per second" << std::endl;
    std::cout << "time : " << travelTime << " seconds" << std::endl;

    delete robot;
    return 0;
}
