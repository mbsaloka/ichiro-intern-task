#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include <cmath>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define MAX_COUNTER 5
#define WHEEL_R 0.0205
#define WHEEL_DIS 0.052

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    DistanceSensor *gs[3];
    char gsNames[3][4] = {"gs0", "gs1", "gs2"};

    for (int i = 0; i < 3; i++) {
        gs[i] = robot->getDistanceSensor(gsNames[i]);
        gs[i]->enable(TIME_STEP);
    }

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

    double dist_value[2] = {0.0, 0.0};
    double wheel_circum = 2 * M_PI * WHEEL_R;
    double encoder_unit = wheel_circum / 6.28;

    double robot_pose[3] = {0.0, 0.0, 0.0};
    double last_ps_value[2] = {0.0, 0.0};

    char current_state = 'f';
    int counter = 0;

    while (robot->step(TIME_STEP) != -1) {
        double gsValues[3];
        for (int i = 0; i < 3; i++) {
            gsValues[i] = gs[i]->getValue();
        }
        bool line_left = gsValues[0] > 600;
        bool line_right = gsValues[2] > 600;

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

        robot_pose[2] += w * dt;
        robot_pose[0] += v * cos(robot_pose[2]) * dt;
        robot_pose[1] += v * sin(robot_pose[2]) * dt;

        for (int i = 0; i < 2; i++) {
            last_ps_value[i] = ps_value[i];
        }

        double leftSpeed = MAX_SPEED;
        double rightSpeed = MAX_SPEED;

        if (current_state == 'f') {
            if (line_right) {
                current_state = 'l';
                counter = 0;
            } else if (line_left) {
                current_state = 'r';
                counter = 0;
            }
        }

        if (current_state == 'l') {
            leftSpeed = 0.4 * MAX_SPEED;
            rightSpeed = 0.8 * MAX_SPEED;
            if (counter >= MAX_COUNTER) {
                current_state = 'f';
            }
        } else if (current_state == 'r') {
            leftSpeed = 0.8 * MAX_SPEED;
            rightSpeed = 0.4 * MAX_SPEED;
            if (counter >= MAX_COUNTER) {
                current_state = 'f';
            }
        }

        counter++;

        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        std::cout << "Position Sensor Value [L, R]: [" << ps_value[0] << " "
                  << ps_value[1] << "]" << std::endl;
        std::cout << "Robot pose: [" << robot_pose[0] << " " << robot_pose[1]
                  << " " << robot_pose[2] << "]" << std::endl;
        std::cout << "--------------------------------" << std::endl;
    };

    delete robot;
    return 0;
}
