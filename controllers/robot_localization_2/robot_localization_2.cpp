// File:          robot_localization_2.cpp
// Date:          26 Jan 2024
// Description:   localization using camera with mcl algorithm (abandoned)
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

#include <cmath>
#include <iomanip>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WHEEL_R 0.0205
#define WHEEL_DIS 0.053

using namespace webots;

void teleop_keyboard(Keyboard kb, Motor *leftMotor, Motor *rightMotor) {
    int key = kb.getKey();
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    // std::cout << "key pressed: " << key << std::endl;

    if (key == 315) {
        leftSpeed = 1.0;
        rightSpeed = 1.0;
    } else if (key == 317) {
        leftSpeed = -1.0;
        rightSpeed = -1.0;
    } else if (key == 316) {
        leftSpeed = 0.5;
        rightSpeed = -0.5;
    } else if (key == 314) {
        leftSpeed = -0.5;
        rightSpeed = 0.5;
    } else {
        leftSpeed = 0.0;
        rightSpeed = 0.0;
    }

    leftMotor->setVelocity(leftSpeed * MAX_SPEED);
    rightMotor->setVelocity(rightSpeed * MAX_SPEED);
}

int main(int argc, char **argv) {
    // Get devices
    Robot *robot = new Robot();
    Camera *camera = robot->getCamera("my_camera");
    Keyboard kb;
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    PositionSensor *leftPositionSensor =
        robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightPositionSensor =
        robot->getPositionSensor("right wheel sensor");

    // Activating devices
    camera->enable(TIME_STEP);
    kb.enable(TIME_STEP);

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0 * MAX_SPEED);
    rightMotor->setVelocity(0.0 * MAX_SPEED);

    leftPositionSensor->enable(TIME_STEP);
    rightPositionSensor->enable(TIME_STEP);

    // Odometry variables
    double dist_value[2] = {0.0, 0.0};
    double wheel_circum = 2 * M_PI * WHEEL_R;
    double encoder_unit = wheel_circum / 6.28;

    double robot_pose[3] = {0.0, 0.0, 0.0};
    double last_ps_value[2] = {0.0, 0.0};

    // Color recognition variables
    unsigned colorBound1[3] = {200, 200, 200};
    unsigned colorBound2[3] = {100, 100, 100};

    while (robot->step(TIME_STEP) != -1) {
        // Calling teleop function
        teleop_keyboard(kb, leftMotor, rightMotor);

        // Camera recognize color
        const unsigned char *image = camera->getImage();
        int width = camera->getWidth();
        int height = camera->getHeight();

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                unsigned red = image[(y * width + x) * 4 + 2];
                unsigned green = image[(y * width + x) * 4 + 1];
                unsigned blue = image[(y * width + x) * 4];
                if (red > colorBound1[0] && green > colorBound1[1] &&
                    blue > colorBound1[2]) {
                    std::cout << "/ ";
                } else if (red > colorBound2[0] && green > colorBound2[1] &&
                           blue > colorBound2[2]) {
                    std::cout << "i ";
                } else if (green < 100) {
                    std::cout << ", ";
                } else {
                    std::cout << "# ";
                }
            }
            std::cout << std::endl;
        }

        // Odometry calculation
        double ps_value[2] = {leftPositionSensor->getValue(),
                              rightPositionSensor->getValue()};
        for (int i = 0; i < 2; i++) {
            double diff = ps_value[i] - last_ps_value[i];
            if (diff < 0.001) {
                diff = 0.0;
                ps_value[i] = last_ps_value[i];
            }
            dist_value[i] = diff * encoder_unit;
        }

        double v = (dist_value[0] + dist_value[1]) / 2.0;
        double w = (dist_value[1] - dist_value[0]) / WHEEL_DIS;
        double dt = 1;

        robot_pose[0] += v * cos(robot_pose[2]) * dt;
        robot_pose[1] += v * sin(robot_pose[2]) * dt;
        robot_pose[2] += w * dt;

        for (int i = 0; i < 2; i++) {
            last_ps_value[i] = ps_value[i];
        }

        std::cout << "Position Sensor Value [L, R]: [" << ps_value[0] << " "
                  << ps_value[1] << "]" << std::endl;
        std::cout << "Robot pose: [" << robot_pose[0] << " " << robot_pose[1]
                  << " " << robot_pose[2] << "]" << std::endl;

        std::cout
            << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
               ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
            << std::endl;
    };

    delete robot;
    return 0;
}
