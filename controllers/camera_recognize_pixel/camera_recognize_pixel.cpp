// File:          camera_recognize_pixel.cpp
// Date:          25 Jan 2024
// Description:   using camera to find specific color on image
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>

#define TIME_STEP 256
#define MAX_SPEED 6.28

using namespace webots;

int main(int argc, char **argv) {
    Robot *robot = new Robot();

    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);

    leftMotor->setVelocity(0.2 * MAX_SPEED);
    rightMotor->setVelocity(0.2 * MAX_SPEED);

    // leftMotor->setVelocity(0.0);
    // rightMotor->setVelocity(0.0);

    Camera *camera = robot->getCamera("my_camera");
    camera->enable(TIME_STEP);

    unsigned colorBound1[3] = {200, 200, 200};
    unsigned colorBound2[3] = {100, 100, 100};

    while (robot->step(TIME_STEP) != -1) {
        // std::string filename = "saveImageTest.png";
        // int quality = 100;  // Kualitas gambar (0-100)
        // int result = camera->saveImage(filename, quality);

        // if (result == -1) {
        //     std::cerr << "Gagal menyimpan gambar ke file " << filename
        //               << std::endl;
        // } else {
        //     std::cout << "Berhasil menyimpan gambar ke file " << filename
        //               << std::endl;
        // }

        const unsigned char *image = camera->getImage();
        int width = camera->getWidth();
        int height = camera->getHeight();

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Dapatkan nilai warna piksel
                unsigned red = image[(y * width + x) * 4 + 2];
                unsigned green = image[(y * width + x) * 4 + 1];
                unsigned blue = image[(y * width + x) * 4];

                // Tentukan warna landmark yang diinginkan (misalnya, merah)
                if (red > colorBound1[0] && green > colorBound1[1] &&
                    blue > colorBound1[2]) {
                    std::cout << "/ ";
                } else if (red > colorBound2[0] && green > colorBound2[1] &&
                           blue > colorBound2[2]) {
                    std::cout << "i ";
                } else {
                    std::cout << "# ";
                }
            }
            std::cout << std::endl;
        }
        std::cout
            << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
               ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
            << std::endl;
        // break;
    };

    delete robot;
    return 0;
}
