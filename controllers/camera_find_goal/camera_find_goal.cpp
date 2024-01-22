// File:          camera_find_goal.cpp
// Date:          22 Jan 2024
// Description:   using camera to find landmark
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 64
#define MAX_SPEED 6.28

using namespace webots;

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
            std::cout << "Number of recognized objects: " << numObjects
                      << std::endl;

            const CameraRecognitionObject &firstObject = recognitionObjects[0];

            // Mengakses informasi posisi objek pada gambar
            const double *objectPosition = firstObject.position;
            const double *objectOrientation = firstObject.orientation;
            const double *objectSize = firstObject.size;
            const double *objectColor = firstObject.colors;

            char *color, *landmarkType;

            // Menentukan warna dan landmark type
            if (objectColor[0] == 1 && objectColor[1] == 1 &&
                objectColor[2] == 0) {
                color = "Yellow";
                landmarkType = "Corner";
            } else if (objectColor[0] == 1 && objectColor[1] == 0 &&
                       objectColor[2] == 1) {
                color = "Purple";
                landmarkType = "Cross";
            } else if (objectColor[0] == 0 && objectColor[1] == 1 &&
                       objectColor[2] == 1) {
                color = "Cyan";
                landmarkType = "T";
            } else if (objectColor[0] == 0 && objectColor[1] == 1 &&
                       objectColor[2] == 0) {
                color = "Green";
                landmarkType = "Goalpost";
            }

            std::cout << "Position: " << objectPosition[0] << " "
                      << objectPosition[1] << std::endl;
            std::cout << "Color [R, G, B]: " << objectColor[0] << ", "
                      << objectColor[1] << ", " << objectColor[2] << " "
                      << color << std::endl;
            std::cout << "Object Detected: " << landmarkType << std::endl;
            std::cout << "--------------------------------------------"
                      << std::endl;
        } else {
            std::cout << "No objects recognized." << std::endl;
        }
    };

    delete robot;
    return 0;
}
