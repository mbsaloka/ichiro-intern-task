// File:          robot_localization.cpp
// Date:          3 Feb 2024
// Description:   localization using camera with mcl algorithm (only 1 landmark)
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
#include <vector>
#include <utility>
#include <random>

#define TIME_STEP 1024
#define MAX_SPEED 6.28
#define WHEEL_R 0.0205
#define WHEEL_DIS 0.052
#define FIELD_WIDTH 450
#define FIELD_LENGTH 600
#define LANDMARK_X 325
#define LANDMARK_Y 0

using namespace webots;

typedef struct recognizedObj_t {
    const double *position;
} RecognizedObject;

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

std::vector<RecognizedObject> camera_recognition(Camera *camera) {
    const CameraRecognitionObject *recognitionObjects =
        camera->getRecognitionObjects();
    int numObjects = camera->getRecognitionNumberOfObjects();
    std::vector<RecognizedObject> object(numObjects);

    if (numObjects > 0 && recognitionObjects) {
        for (int i = 0; i < numObjects; i++) {
            const CameraRecognitionObject &curObject = recognitionObjects[i];
            object[i].position = curObject.position;
        }
    }

    return object;
}

void print_recognized_objects(std::vector<RecognizedObject> object) {
    int numObjects = object.size();
    if (numObjects > 0) {
        std::cout << "Number of recognized objects: " << numObjects
                  << std::endl;

        std::cout << "Position [x, y] :" << std::endl;

        for (int i = 0; i < numObjects; i++) {
            std::cout << std::fixed << std::setprecision(5)
                      << object[i].position[0] * 100 << std::setw(10)
                      << std::fixed << std::setprecision(5)
                      << object[i].position[1] * 100 << std::endl;
        }
    } else {
        std::cout << "No objects recognized." << std::endl;
    }
    std::cout << "-------------------------------------------" << std::endl;
}

// Function to calculate individual likelihood for one object
double calculate_likelihood(const RecognizedObject &measurement,
                            const std::pair<double, double> &particle) {
    double sigma_x = 1.0;
    double sigma_y = 1.0;

    double relative_position_x = LANDMARK_X - particle.first;
    double relative_position_y = LANDMARK_Y - particle.second;

    double exponent =
        -0.5 * (pow((measurement.position[0] * 100 - relative_position_x), 2) /
                    pow(sigma_x, 2) +
                pow((measurement.position[1] * 100 - relative_position_y), 2) /
                    pow(sigma_y, 2));

    return exp(exponent) / (2 * M_PI * sigma_x * sigma_y);
}

// Function for resampling particles
void resample_particles(std::vector<std::pair<double, double>> &particles,
                        std::vector<double> &weights) {
    std::vector<std::pair<double, double>> new_particles;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> dist(weights.begin(), weights.end());

    for (size_t i = 0; i < particles.size(); ++i) {
        int index = dist(gen);
        new_particles.push_back(particles[index]);
    }

    particles = new_particles;
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
    camera->recognitionEnable(TIME_STEP);
    kb.enable(TIME_STEP);

    if (camera->hasRecognition()) {
        std::cout << "Has camera recognition" << std::endl;
    } else {
        std::cout << "Not" << std::endl;
    }

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

    // Landmark pose in cm
    // double landmark[14][2] = {
    //     {0.0, 0.0},     {0.0, 80.0},     {0.0, -80.0},   {0.0, 30.0},
    //     {0.0, -30.0},   {320.0, 0.0},    {380.0, 100.0}, {380.0, -100.0},
    //     {450.0, 100.0}, {450.0, -100.0}, {450.0, 300.0}, {450.0, -300.0},
    //     {450.0, 80.0},  {450.0, -80.0},
    // };

    // MCL variables
    int num_particles = FIELD_WIDTH * FIELD_LENGTH;
    std::vector<std::pair<double, double>> particles;
    std::vector<double> weights;

    for (int i = 0; i < FIELD_WIDTH; ++i) {
        for (int j = -FIELD_LENGTH / 2; j < FIELD_LENGTH / 2; ++j) {
            std::pair<double, double> particle;
            particle = {i, j};
            particles.push_back(particle);
            weights.push_back(1.0 / num_particles);
        }
    }

    while (robot->step(TIME_STEP) != -1) {
        // Calling teleop function
        teleop_keyboard(kb, leftMotor, rightMotor);

        // Camera recognize object
        std::vector<RecognizedObject> recognized_object =
            camera_recognition(camera);

        print_recognized_objects(recognized_object);

        // Odometry calculation
        double ps_value[2] = {leftPositionSensor->getValue(),
                              rightPositionSensor->getValue()};
        for (int i = 0; i < 2; i++) {
            double diff = ps_value[i] - last_ps_value[i];
            if (diff < 0.001 && diff > -0.001) {
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

        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
                     ">>>>>>>>"
                  << std::endl;

        // MCL algorithm
        // Loop through each particle
        for (int i = 0; i < num_particles; ++i) {
            double likelihood = 0.0;
            if (recognized_object.size() > 0) {
                // Likelihood evaluation
                likelihood =
                    calculate_likelihood(recognized_object[0], particles[i]);
            }

            // Assign likelihood as weight to the particle
            weights[i] = likelihood;
        }

        // Normalize weights
        double sum_weights = 0.0;
        for (int i = 0; i < num_particles; ++i) {
            sum_weights += weights[i];
        }

        for (int i = 0; i < num_particles; ++i) {
            weights[i] /= sum_weights;
        }

        // Print particles
        double sum_samples = 0.0;
        double best_sample_weight = 0.0;
        int best_sample_index = 0;
        for (int i = 0; i < num_particles; i++) {
            if (weights[i] > 0.00001) {
                std::cout << "Particle " << std::setw(5) << i
                          << "  weight: " << std::fixed << std::setprecision(5)
                          << weights[i] << std::setw(5) << " [" << std::fixed
                          << std::setprecision(2) << particles[i].first << ", "
                          << std::fixed << std::setprecision(2)
                          << particles[i].second << "]" << std::endl;
                sum_samples += weights[i];

                if (weights[i] > best_sample_weight) {
                    best_sample_weight = weights[i];
                    best_sample_index = i;
                }
            }
        }

        std::cout << "Sum weights: " << sum_samples << std::endl;
        std::cout << "Best sample: " << best_sample_weight << " (particle num "
                  << best_sample_index << " [" << std::fixed
                  << std::setprecision(2) << particles[best_sample_index].first
                  << ", " << std::fixed << std::setprecision(2)
                  << particles[best_sample_index].second << "])" << std::endl;

        // Resampling particles based on weights
        // resample_particles(particles, weights);

        // std::cout << "-------------------------------------------" <<
        // std::endl; Print resampled particles for (int i = 0; i <
        // num_particles; i++) {
        //     if (weights[i] > 0.00001) {
        //         std::cout << "Particle " << std::setw(5) << i
        //                   << "  weight: " << std::fixed <<
        //                   std::setprecision(5)
        //                   << weights[i] << std::setw(5) << " [" << std::fixed
        //                   << std::setprecision(2) << particles[i].first << ",
        //                   "
        //                   << std::fixed << std::setprecision(2)
        //                   << particles[i].second << "]" << std::endl;
        //     }
        // }
        // std::cout << "Sum weights: " << sum_samples << std::endl;
        // std::cout << "Best sample: " << best_sample_weight << " (particle num
        // "
        //           << best_sample_index << " [" << std::fixed
        //           << std::setprecision(2) <<
        //           particles[best_sample_index].first
        //           << ", " << std::fixed << std::setprecision(2)
        //           << particles[best_sample_index].second << "])" <<
        //           std::endl;

        // Your resampled particles are now in the particle_predictions vector
        break;
    };

    delete robot;
    return 0;
}
