#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define MAX_COUNTER 5

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

    char current_state = 'f';
    int counter = 0;

    while (robot->step(TIME_STEP) != -1) {
        double gsValues[3];
        for (int i = 0; i < 3; i++) {
            gsValues[i] = gs[i]->getValue();
        }
        bool line_left = gsValues[0] > 600;
        bool line_right = gsValues[2] > 600;

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

        std::cout << "Left sensor : " << gsValues[0] << std::endl;
        std::cout << "Center sensor : " << gsValues[1] << std::endl;
        std::cout << "Right sensor : " << gsValues[2] << std::endl;
        std::cout << "STATUS : "
                  << ((current_state == 'f')   ? "Forward"
                      : (current_state == 'r') ? "right"
                                               : "left")
                  << std::endl;
        std::cout << "Counter : " << counter << std::endl;
        std::cout << "-------------------------" << std::endl;
    };

    delete robot;
    return 0;
}
