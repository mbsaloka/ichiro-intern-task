// File:          robot_localization.cpp
// Date:          18 Feb 2024
// Description:   localization using camera with mcl algorithm (implement random
//                distribution)
// Author:        mbsaloka Modifications: activate angular localization

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
#include <random>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define WHEEL_R 0.020
#define WHEEL_DIS 0.052
// #define FIELD_WIDTH 450
// #define FIELD_LENGTH 600
#define FIELD_WIDTH 50
#define FIELD_LENGTH 100
#define NUM_ANGLE 24
#define START_X 0
#define START_Y 0
#define START_W 0
#define VAR_X 10
#define VAR_Y 10
#define VAR_W 0.05
#define N_PARTICLE 1000
#define LB_WEIGHT 0.000001

using namespace webots;

double w_fast = 0.0;
double w_slow = 0.0;
double a_fast = 0.5;
double a_slow = 0.0005;
double max_weight;

typedef struct recognizedObj_t {
    const double *position;
} RecognizedObject;

typedef struct particle_t {
    double base_x;
    double base_y;
    double base_theta;
    double x;
    double y;
    double theta;
    double weight;
} Particle;

// Landmark pose in cm
double LANDMARK[14][2] = {
    {0.0, 0.0},     {0.0, 80.0},     {0.0, -80.0},   {0.0, 300.0},
    {0.0, -300.0},  {240.0, 0.0},    {350.0, 250.0}, {350.0, -250.0},
    {450.0, 250.0}, {450.0, -250.0}, {450.0, 300.0}, {450.0, -300.0},
    {450.0, 130.0}, {450.0, -130.0},
};

void teleop_keyboard(Keyboard kb, Motor *leftMotor, Motor *rightMotor,
                     bool *firstIteration) {
    int key = kb.getKey();
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

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
    } else if (key == 'r' || key == 'R') {
        *firstIteration = true;
    } else {
        leftSpeed = 0.0;
        rightSpeed = 0.0;
    }

    leftMotor->setVelocity(leftSpeed * MAX_SPEED);
    rightMotor->setVelocity(rightSpeed * MAX_SPEED);
}

double get_random_number() {
    // Random Generators
    std::random_device rd;
    std::mt19937 gen(rd());
    // get random number from a uniform distribution
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
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
    for (int i = 0; i < 14; i++) {
        double relative_position_x = particle.x + measurement.position[0] * 100;
        double relative_position_y = particle.y + measurement.position[1] * 100;

        double dx = relative_position_x - particle.x;
        double dy = relative_position_y - particle.y;

        double x_rot = dx * cos(particle.theta) - dy * sin(particle.theta);
        double y_rot = dx * sin(particle.theta) + dy * cos(particle.theta);

        relative_position_x = x_rot + particle.x;
        relative_position_y = y_rot + particle.y;

        double exponent =
            -0.5 *
            (pow((LANDMARK[i][0] - relative_position_x), 2) / pow(sigma_x, 2) +
             pow((LANDMARK[i][1] - relative_position_y), 2) / pow(sigma_y, 2));

        double likelihood = exp(exponent) / (2 * M_PI * sigma_x * sigma_y);

        if (likelihood > best_likelihood) {
            best_likelihood = likelihood;
        }
    }

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

// Generate first gen particles
void init_particles(std::vector<Particle> &particles, bool init = false) {
    std::vector<Particle> new_particles;

    std::random_device xrd, yrd, wrd;
    if (init) {
        std::cout << "-------------------FIRST INIT---------------------"
                  << std::endl;
        std::normal_distribution<double> xrg(START_X, VAR_X),
            yrg(START_Y, VAR_Y), wrg(START_W, VAR_W);
        std::vector<Particle> new_particles;
        for (int i = 0; i < N_PARTICLE; i++) {
            Particle p;
            p.base_x = xrg(xrd);
            p.base_y = yrg(yrd);
            p.base_theta = wrg(wrd);
            p.x = p.base_x;
            p.y = p.base_y;
            p.theta = p.base_theta;
            p.weight = 1.0 / N_PARTICLE;

            new_particles.push_back(p);
        }
        particles = new_particles;
    } else {
        std::cout << "---------------------RESTART-----------------------"
                  << std::endl;
        std::uniform_real_distribution<double> xrg(0, FIELD_WIDTH),
            yrg(-FIELD_LENGTH / 2, FIELD_LENGTH / 2), wrg(-3.14, 3.14);
        std::vector<Particle> new_particles;

        for (int i = 0; i < N_PARTICLE; i++) {
            Particle p;
            p.base_x = xrg(xrd);
            p.base_y = yrg(yrd);
            p.base_theta = wrg(wrd);
            p.x = p.base_x;
            p.y = p.base_y;
            p.theta = p.base_theta;
            p.weight = 1.0 / N_PARTICLE;

            new_particles.push_back(p);
        }
        particles = new_particles;

        // for (auto &p : particles) {
        //     p.base_x = xrg(xrd);
        //     p.base_y = yrg(yrd);
        //     p.base_theta = wrg(wrd);
        //     p.x = p.base_x;
        //     p.y = p.base_y;
        //     p.theta = p.base_theta;
        //     p.weight = 1.0 / N_PARTICLE;
        // }
    }
}

// Function for resampling particles
void resample_particles(std::vector<Particle> &particles) {
    // Method 4
    // /*
    std::vector<Particle> new_particles;

    for (size_t i = 0; i < particles.size(); ++i) {
        if (particles[i].weight > LB_WEIGHT) {
            new_particles.push_back(particles[i]);
        }
    }

    // particles = new_particles;
    // */

    /* // Method 3
    std::vector<Particle> p = particles;
    // get random index
    int index = get_random_number() * particles.size();
    double beta = 0.0;
    for (int i = 0; i < particles.size(); ++i) {
        beta += get_random_number() * 2 * max_weight;
        while (beta > particles[index].weight) {
            beta -= particles[index].weight;
            index = (index + 1) % particles.size();
        }
        // std::cout << index << std::endl;
        //  select particle at index
        p[i] = particles[index];
        // std::cout << p[i].r.get_pose() << std::endl;
    }
    // return new particle set
    particles = p;
    */

    /* // Method 2
    std::vector<Particle> plist;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rg(0.0, 1.0 / N_PARTICLE);
    double r = rg(gen);
    // double c = vis_weight(particles[0]);
    double c = particles[0].weight;
    int idx = 0;
    std::random_device rd1;
    std::mt19937 gen1(rd1());
    std::random_device rd2;
    std::mt19937 gen2(rd2());
    std::uniform_real_distribution<double> xrg(0, 450), yrg(-300, 300),
        wrg(-3.14, 3.14);
    double random_prob = (1.0 - (w_fast / w_slow));
    // std::bernoulli_distribution random_gen((random_prob<0) ? 0.0 :
    // (random_prob>1 ? 1.0 : random_prob));
    std::bernoulli_distribution random_gen(std::max(0.0, random_prob));
    for (int i = 0; i < N_PARTICLE; i++) {
        if (random_gen(gen1)) {
            Particle p;
            p.base_x = xrg(gen2);
            p.base_y = yrg(gen2);
            p.base_theta = wrg(gen2);
            p.x = p.base_x;
            p.y = p.base_y;
            p.theta = p.base_theta;
            p.weight = 1.0 / N_PARTICLE;
            plist.push_back(p);
        } else {
            double u = r + ((double)i / N_PARTICLE);
            while (u > c) {
                idx += 1;
                // c += vis_weight(particles[idx]);
                c += particles[idx].weight;
            }
            plist.push_back(particles[idx]);
        }
    }
    particles = plist;
    */

    // Method 1
    /*
    std::vector<Particle> new_particles;

    for (size_t i = 0; i < particles.size(); ++i) {
        if (particles[i].weight > 0.00001) {
            new_particles.push_back(particles[i]);
        }
    }

    particles = new_particles;
    */
}

// Print particles
void print_particles(std::vector<Particle> &particles, int num_particles) {
    double sum_samples = 0.0;
    double best_sample_weight = 0.0;
    int best_sample_index = 0;
    for (int i = 0; i < num_particles; i++) {
        if (particles[i].weight > LB_WEIGHT) {
            std::cout << "Particle " << std::setw(5) << i
                      << "  weight: " << std::fixed << std::setprecision(5)
                      << particles[i].weight << std::setw(5) << " ["
                      << std::fixed << std::setprecision(2) << particles[i].x
                      << ", " << std::fixed << std::setprecision(2)
                      << particles[i].y << ", " << std::fixed
                      << std::setprecision(2) << particles[i].theta << "]"
                      << std::endl;

            if (particles[i].weight > best_sample_weight) {
                best_sample_weight = particles[i].weight;
                best_sample_index = i;
            }
        }
        sum_samples += particles[i].weight;
    }

    max_weight = best_sample_weight;

    std::cout << "Sum weights: " << sum_samples << std::endl;
    std::cout << "Best sample: " << std::fixed << std::setprecision(5)
              << best_sample_weight << " (particle num " << best_sample_index
              << " [" << std::fixed << std::setprecision(2)
              << particles[best_sample_index].x << ", " << std::fixed
              << std::setprecision(2) << particles[best_sample_index].y << ", "
              << std::fixed << std::setprecision(2)
              << particles[best_sample_index].theta << "])" << std::endl;
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
    double last_rotation = 0.0;
    double timestep = robot->getBasicTimeStep();

    // MCL variables
    // int num_particles = FIELD_WIDTH * FIELD_LENGTH * NUM_ANGLE;
    int num_particles = N_PARTICLE;
    std::vector<Particle> particles;
    std::vector<double> weights;

    // Timer
    auto lastTime = std::chrono::high_resolution_clock::now();

    // Flag for first iteration state
    bool veryFirstIteration = true;
    bool firstIteration = true;
    int iteration = 0;

    // Init particles with random normal distribution
    std::random_device x_rd, y_rd, w_rd;
    std::uniform_real_distribution<double> x_rgen(0, FIELD_WIDTH),
        y_rgen(-FIELD_LENGTH / 2, FIELD_LENGTH / 2), w_rgen(-3.14, 3.14);
    for (int i = 0; i < N_PARTICLE; i++) {
        Particle new_particle;
        new_particle.base_x = x_rgen(x_rd);
        new_particle.base_y = y_rgen(y_rd);
        new_particle.base_theta = w_rgen(w_rd);
        new_particle.x = new_particle.base_x;
        new_particle.y = new_particle.base_y;
        new_particle.theta = new_particle.base_theta;
        new_particle.weight = 1.0 / N_PARTICLE;

        // std::cout << "[DEBUG] Particle " << i << " (" << std::fixed
        //           << std::setprecision(3) << new_particle.x << " " <<
        //           std::fixed
        //           << std::setprecision(3) << new_particle.y << " " <<
        //           std::fixed
        //           << std::setprecision(3) << new_particle.theta << ")"
        //           << std::endl;

        particles.push_back(new_particle);
    }

    while (robot->step(TIME_STEP) != -1) {
        // Calling teleop function
        teleop_keyboard(kb, leftMotor, rightMotor, &firstIteration);

        // Init particles
        if (firstIteration) {
            robot_pose[0] = 0.0;
            robot_pose[1] = 0.0;
            robot_pose[2] = 0.0;
            init_particles(particles, veryFirstIteration);
            veryFirstIteration = false;
            num_particles = N_PARTICLE;
        }

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
        if (firstIteration) {
            last_rotation = imu->getRollPitchYaw()[2];
        }
        double w = (imu->getRollPitchYaw()[2] - last_rotation) / timestep;
        last_rotation = imu->getRollPitchYaw()[2];
        double dt = 1;

        robot_pose[0] += v * cos(robot_pose[2]) * dt;
        robot_pose[1] += v * sin(robot_pose[2]) * dt;
        robot_pose[2] += w * timestep;
        // robot_pose[2] = imu->getRollPitchYaw()[2];

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
                particles[i].theta = particles[i].base_theta + robot_pose[2];
            }

            if (num_particles > 10) {
                for (int i = 0; i < num_particles; ++i) {
                    double likelihood = 0.0;
                    if (recognized_object.size() > 0) {
                        // Likelihood evaluation
                        likelihood = calculate_total_likelihood(
                            recognized_object, particles[i]);
                    }

                    // Assign likelihood as weight to the particle
                    particles[i].weight = likelihood;
                }

                // Normalize weights
                double sum_weights = 0.0;
                for (int i = 0; i < num_particles; ++i) {
                    sum_weights += particles[i].weight;
                    // std::cout << "[likelihood] " << i << ": "
                    //           << particles[i].weight << std::endl;
                }
                if (sum_weights > 0.0) {
                    for (int i = 0; i < num_particles; ++i) {
                        particles[i].weight /= sum_weights;
                    }
                } else {
                    for (int i = 0; i < num_particles; ++i) {
                        particles[i].weight = 1.0 / num_particles;
                    }
                }
                double w_avg = 0.0;
                w_avg = sum_weights / N_PARTICLE;
                w_slow = w_slow + a_slow * (w_avg - w_slow);
                w_fast = w_fast + a_fast * (w_avg - w_fast);
            }

            // Print particles
            std::cout << iteration << " iteration" << std::endl;
            print_particles(particles, num_particles);
            std::cout << "Num particles: " << num_particles << std::endl;

            // Resampling particles based on weights
            resample_particles(particles);
            num_particles = particles.size();
        } else if (num_particles == 0) {
            init_particles(particles);
            num_particles = N_PARTICLE;
        } else {
            std::cout << iteration << " iteration" << std::endl;
            print_particles(particles, num_particles);
            std::cout << "Num particles: " << num_particles << std::endl;
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