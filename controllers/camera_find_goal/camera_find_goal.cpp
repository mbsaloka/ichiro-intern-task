// File:          camera_find_goal.cpp
// Date:          22 Jan 2024
// Description:   using camera to find landmark
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <iomanip>

#define TIME_STEP 64
#define MAX_SPEED 6.28

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

    leftMotor->setVelocity(0.05 * MAX_SPEED);
    rightMotor->setVelocity(-0.05 * MAX_SPEED);

    Camera *camera = robot->getCamera("my_camera");
    camera->enable(TIME_STEP);
    camera->recognitionEnable(TIME_STEP);

    if (camera->hasRecognition()) {
        std::cout << "Has camera recognition" << std::endl;
    } else {
        std::cout << "Not" << std::endl;
    }

    while (robot->step(TIME_STEP) != -1) {
        const CameraRecognitionObject *recognitionObjects =
            camera->getRecognitionObjects();

        int numObjects = camera->getRecognitionNumberOfObjects();

        // Memeriksa apakah ada objek yang dikenali
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

            std::cout << "---------------------------------------------------"
                      << std::endl;
        } else {
            std::cout << "No objects recognized." << std::endl;
        }
    };

    delete robot;
    return 0;
}
