// File:          robot_localization.cpp
// Date:          23 Jan 2024
// Description:   localization using camera with mcl algorithm
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>

#include <cmath>
#include <iomanip>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WHEEL_R 0.0205
#define WHEEL_DIS 0.053

using namespace webots;

typedef struct obj_t {
    const double *position;
    const double *size;
    const double *orientation;
    const double *colorRGB;
    const char *landmarkType;
    const char *color;
} RecognizedObject;

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.5 * MAX_SPEED);
    rightMotor->setVelocity(0.5 * MAX_SPEED);

    Camera *camera = robot->getCamera("my_camera");
    camera->enable(TIME_STEP);
    camera->recognitionEnable(TIME_STEP);

    if (camera->hasRecognition()) {
        std::cout << "Has camera recognition" << std::endl;
    } else {
        std::cout << "Not" << std::endl;
    }

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

    while (robot->step(TIME_STEP) != -1) {
        // CAMERA RECOGNIZE
        const CameraRecognitionObject *recognitionObjects =
            camera->getRecognitionObjects();

        int numObjects = camera->getRecognitionNumberOfObjects();

        if (numObjects > 0 && recognitionObjects) {
            RecognizedObject object[numObjects];

            for (int i = 0; i < numObjects; i++) {
                const CameraRecognitionObject &curObject =
                    recognitionObjects[i];

                object[i].position = curObject.position;
                object[i].orientation = curObject.orientation;
                object[i].size = curObject.size;
                object[i].colorRGB = curObject.colors;

                if (object[i].colorRGB[0] == 1 && object[i].colorRGB[1] == 1 &&
                    object[i].colorRGB[2] == 0) {
                    object[i].color = "Yellow";
                    object[i].landmarkType = "Corner";
                } else if (object[i].colorRGB[0] == 1 &&
                           object[i].colorRGB[1] == 0 &&
                           object[i].colorRGB[2] == 1) {
                    object[i].color = "Purple";
                    object[i].landmarkType = "Cross";
                } else if (object[i].colorRGB[0] == 0 &&
                           object[i].colorRGB[1] == 1 &&
                           object[i].colorRGB[2] == 1) {
                    object[i].color = "Cyan";
                    object[i].landmarkType = "T";
                } else if (object[i].colorRGB[0] == 0 &&
                           object[i].colorRGB[1] == 1 &&
                           object[i].colorRGB[2] == 0) {
                    object[i].color = "Green";
                    object[i].landmarkType = "Goalpost";
                }
            }

            std::cout << "Number of recognized objects: " << numObjects
                      << std::endl;
            for (int i = 0; i < numObjects; i++) {
                if (i == 0) {
                    std::cout << "[";
                }

                std::cout << object[i].landmarkType;

                if (i == numObjects - 1) {
                    std::cout << ']' << std::endl;
                } else {
                    std::cout << ", ";
                }
            }

            std::cout << std::setw(17) << "Object Type |" << std::setw(12)
                      << "Color |" << std::setw(20) << "Position [x, y]"
                      << std::endl;

            for (int i = 0; i < numObjects; i++) {
                std::cout << std::setw(15) << object[i].landmarkType << " |"
                          << std::setw(10) << object[i].color << " |"
                          << std::setw(10) << std::fixed << std::setprecision(5)
                          << object[i].position[0] << std::setw(10)
                          << std::fixed << std::setprecision(5)
                          << object[i].position[1] << std::endl;
            }
            std::cout << ">>>" << std::endl;
        } else {
            std::cout << "No objects recognized." << std::endl;
        }

        // ODOMETRY
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
        std::cout << "--------------------------------" << std::endl;
    };

    delete robot;
    return 0;
}
