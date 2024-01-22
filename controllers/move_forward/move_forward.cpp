// File:          move_forward.cpp
// Date:          16 Jan 2024
// Description:   basic move on e-puck robots
// Author:        mbsaloka
// Modifications:

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");

    leftMotor->setPosition(-10.0);
    rightMotor->setPosition(-10.0);

    leftMotor->setVelocity(MAX_SPEED);
    rightMotor->setVelocity(MAX_SPEED);

    // leftMotor->setVelocity(0.0);
    // rightMotor->setVelocity(0.0);

    // leftMotor->setVelocity(-1.0 * MAX_SPEED);
    // rightMotor->setVelocity(-1.0 * MAX_SPEED);

    while (robot->step(TIME_STEP) != -1) {
    };

    delete robot;
    return 0;
}
