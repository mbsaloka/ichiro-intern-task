// File:          distance_sensor_1.cpp
// Date:          17 Jan 2024
// Description:   proportional motor control using distance sensor
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    DistanceSensor *frontDistanceSensor = robot->getDistanceSensor("front_ds");
    frontDistanceSensor->enable(TIME_STEP);

    while (robot->step(TIME_STEP) != -1) {
        double frontValue = frontDistanceSensor->getValue();
        double newSpeed = MAX_SPEED * (frontValue - 100) / 900;

        if (newSpeed > MAX_SPEED) {
            newSpeed = MAX_SPEED;
        } else if (newSpeed < -MAX_SPEED) {
            newSpeed = -MAX_SPEED;
        }

        std::cout << "Distance Sensor : " << frontValue << std::endl;
        std::cout << "Motor Speed : " << newSpeed << std::endl;
        std::cout << "------------------------" << std::endl;

        leftMotor->setVelocity(newSpeed);
        rightMotor->setVelocity(newSpeed);

    }

    delete robot;
    return 0;
}
