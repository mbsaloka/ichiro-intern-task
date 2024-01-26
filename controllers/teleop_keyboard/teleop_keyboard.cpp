// File:          teleop_keyboard.cpp
// Date:          26 Jan 2024
// Description:   move e-puck using keyboard
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>

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

    Keyboard kb;

    kb.enable(TIME_STEP);
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    while (robot->step(TIME_STEP) != -1) {
        int key = kb.getKey();
        // std::cout << "key pressed: " << key << std::endl;

        if (key == 315) {
            leftSpeed = 1.0;
            rightSpeed = 1.0;
        } else if (key == 317) {
            leftSpeed = -1.0;
            rightSpeed = -1.0;
        } else if (key == 316) {
            leftSpeed = 1.0;
            rightSpeed = -1.0;
        } else if (key == 314) {
            leftSpeed = -1.0;
            rightSpeed = 1.0;
        } else {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
        }

        leftMotor->setVelocity(leftSpeed * MAX_SPEED);
        rightMotor->setVelocity(rightSpeed * MAX_SPEED);
    };

    delete robot;
    return 0;
}
