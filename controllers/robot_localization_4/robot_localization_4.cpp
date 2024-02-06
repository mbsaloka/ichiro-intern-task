// File:          robot_localization.cpp
// Date:          5 Feb 2024
// Description:   localization using camera with mcl algorithm (all landmark and
//                using imu for rotation)
// Author:        mbsaloka
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>

#include <cmath>
#include <iomanip>
#include <vector>
#include <utility>
#include <chrono>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WHEEL_R 0.020
#define WHEEL_DIS 0.052
#define FIELD_WIDTH 450
#define FIELD_LENGTH 600

using namespace webots;

typedef struct recognizedObj_t {
    const double *position;
} RecognizedObject;

typedef struct particle_t {
    double base_x;
    double base_y;
    double x;
    double y;
    double theta;
} Particle;

// Landmark pose in cm
double LANDMARK[14][2] = {
    {0.0, 0.0},     {0.0, 80.0},     {0.0, -80.0},   {0.0, 300.0},
    {0.0, -300.0},  {325.0, 0.0},    {395.0, 110.0}, {395.0, -110.0},
    {450.0, 110.0}, {450.0, -110.0}, {450.0, 300.0}, {450.0, -300.0},
    {450.0, 80.0},  {450.0, -80.0},
};

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
double calculate_object_likelihood(const RecognizedObject &measurement,
                                   const Particle &particle) {
    double sigma_x = 1.0;
    double sigma_y = 1.0;

    double best_likelihood = 0.0;
    // int best_index = -1;
    for (int i = 0; i < 14; i++) {
        double relative_position_x = particle.x + measurement.position[0] * 100;
        double relative_position_y = particle.y + measurement.position[1] * 100;

        double exponent =
            -0.5 *
            (pow((LANDMARK[i][0] - relative_position_x), 2) / pow(sigma_x, 2) +
             pow((LANDMARK[i][1] - relative_position_y), 2) / pow(sigma_y, 2));

        double likelihood = exp(exponent) / (2 * M_PI * sigma_x * sigma_y);

        if (likelihood > best_likelihood) {
            best_likelihood = likelihood;
            // best_index = i;
        }

        // double log_likelihood =
        //     -0.5 *
        //     (log(2 * M_PI * sigma_x * sigma_y) +
        //      pow((LANDMARK[i][0] - relative_position_x), 2) / pow(sigma_x, 2)
        //      + pow((LANDMARK[i][1] - relative_position_y), 2) / pow(sigma_y,
        //      2));

        // if (log_likelihood > best_likelihood) {
        //     best_likelihood = log_likelihood;
        //     // best_index = i;
        // }
        // std::cout << "log_likelihood [" << i << "] : " << log_likelihood << "
        // ("
        //           << LANDMARK[i][0] << ", " << LANDMARK[i][1] << ")"
        // << std::endl;
    }
    // std::cout << "best_likelihood : " << best_likelihood << " ("
    //           << measurement.position[0] << ", " << measurement.position[1]
    //           << ") -> [" << best_index << "]" << std::endl;
    // std::cout << "-------------------------------------------" << std::endl;

    return best_likelihood;
}

double calculate_total_likelihood(
    const std::vector<RecognizedObject> &object_measurements,
    const Particle &particle) {
    double total_likelihood = 1.0;

    for (const auto &object_measurement : object_measurements) {
        total_likelihood *=
            calculate_object_likelihood(object_measurement, particle);
    }

    return total_likelihood;
}

// Function for resampling particles
void resample_particles(std::vector<Particle> &particles,
                        std::vector<double> &weights) {
    std::vector<Particle> new_particles;
    std::vector<double> new_weights;

    for (size_t i = 0; i < particles.size(); ++i) {
        if (weights[i] > 0.001) {
            new_particles.push_back(particles[i]);
            new_weights.push_back(weights[i]);
        }
    }

    particles = new_particles;
    weights = new_weights;
}

// Print particles
void print_particles(std::vector<Particle> &particles,
                     std::vector<double> &weights, int num_particles) {
    double sum_samples = 0.0;
    double best_sample_weight = 0.0;
    int best_sample_index = 0;
    for (int i = 0; i < num_particles; i++) {
        if (weights[i] > 0.00001) {
            std::cout << "Particle " << std::setw(5) << i
                      << "  weight: " << std::fixed << std::setprecision(5)
                      << weights[i] << std::setw(5) << " [" << std::fixed
                      << std::setprecision(2) << particles[i].x << ", "
                      << std::fixed << std::setprecision(2) << particles[i].y
                      << "]" << std::endl;
            sum_samples += weights[i];

            if (weights[i] > best_sample_weight) {
                best_sample_weight = weights[i];
                best_sample_index = i;
            }
        }
    }

    std::cout << "Sum weights: " << sum_samples << std::endl;
    std::cout << "Best sample: " << best_sample_weight << " (particle num "
              << best_sample_index << " [" << std::fixed << std::setprecision(2)
              << particles[best_sample_index].x << ", " << std::fixed
              << std::setprecision(2) << particles[best_sample_index].y << "])"
              << std::endl;
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
    InertialUnit *imu = robot->getInertialUnit("inertial unit");

    // Activating devices
    camera->enable(TIME_STEP);
    camera->recognitionEnable(TIME_STEP);
    kb.enable(TIME_STEP);
    imu->enable(TIME_STEP);

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

    // MCL variables
    int num_particles = FIELD_WIDTH * FIELD_LENGTH;
    std::vector<Particle> particles;
    std::vector<double> weights;

    for (int i = 0; i < FIELD_WIDTH; ++i) {
        for (int j = -FIELD_LENGTH / 2; j < FIELD_LENGTH / 2; ++j) {
            Particle particle;
            particle.base_x = i;
            particle.base_y = j;
            particle.x = i;
            particle.y = j;
            particle.theta = 0.0;
            particles.push_back(particle);
            weights.push_back(1.0 / num_particles);
        }
    }

    // Timer
    auto lastTime = std::chrono::high_resolution_clock::now();

    // Flag for first iteration state
    bool firstIteration = true;
    int iteration = 0;

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
        // double w = (dist_value[1] - dist_value[0]) / WHEEL_DIS;
        double dt = 1;

        robot_pose[0] += v * cos(robot_pose[2]) * dt;
        robot_pose[1] += v * sin(robot_pose[2]) * dt;
        // robot_pose[2] += w * dt;
        robot_pose[2] = -imu->getRollPitchYaw()[2];

        for (int i = 0; i < 2; i++) {
            last_ps_value[i] = ps_value[i];
        }

        // Timer Update
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = currentTime - lastTime;
        double seconds = duration.count();

        // MCL algorithm
        // Loop through each particle
        if (seconds >= 1.0 || firstIteration) {
            firstIteration = false;
            lastTime = currentTime;
            iteration++;

            for (int i = 0; i < num_particles; ++i) {
                particles[i].x = particles[i].base_x + robot_pose[0] * 100;
                particles[i].y = particles[i].base_y + robot_pose[1] * 100;
                particles[i].theta = robot_pose[2];
            }

            if (num_particles > 3) {
                for (int i = 0; i < num_particles; ++i) {
                    double likelihood = 0.0;
                    if (recognized_object.size() > 0) {
                        // Likelihood evaluation
                        likelihood = calculate_total_likelihood(
                            recognized_object, particles[i]);
                    }

                    // Assign likelihood as weight to the particle
                    weights[i] = likelihood;
                    // std::cout << "Likelihood: " << likelihood << std::endl;

                    // std::cout << "Particle " << std::setw(5) << i
                    //           << "  weight: " << std::fixed <<
                    //           std::setprecision(5)
                    //           << weights[i] << std::setw(5) << " [" <<
                    //           std::fixed
                    //           << std::setprecision(2) << particles[i].first
                    //           << ",
                    //           "
                    //           << particles[i].second << "]" << std::endl;
                }

                // Normalize weights
                double sum_weights = 0.0;
                for (int i = 0; i < num_particles; ++i) {
                    sum_weights += weights[i];
                }

                for (int i = 0; i < num_particles; ++i) {
                    weights[i] /= sum_weights;
                }
            }
            // Print particles
            std::cout << iteration << " iteration" << std::endl;
            print_particles(particles, weights, num_particles);

            // Resampling particles based on weights
            resample_particles(particles, weights);
            num_particles = particles.size();
        } else {
            std::cout << iteration << " iteration" << std::endl;
            print_particles(particles, weights, num_particles);
        }

        // Print odometry
        std::cout << "-------------------------------------------" << std::endl;

        std::cout << "Position Sensor Value [L, R]: [" << ps_value[0] << " "
                  << ps_value[1] << "]" << std::endl;
        std::cout << "Robot pose: [" << robot_pose[0] << " " << robot_pose[1]
                  << " " << robot_pose[2] << "]" << std::endl;

        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
                  << std::endl;
    };

    delete robot;
    return 0;
}
