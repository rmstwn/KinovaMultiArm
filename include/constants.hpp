/*
Author(s): Muhammad Ramadhan Hadi Setyawan
Institute: Takesue Laboratory, Tokyo Metropolitan University
Description: Description of the file/module

Copyright (c) 2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_
#include <Eigen/Core>
#include <vector>
#include <stdlib.h> /* abs */
#include <unistd.h>
#include <cmath>

#define DEG_TO_RAD(x) (x) * 3.14159265358979323846 / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / 3.14159265358979323846

enum integration_method 
{
    PREDICTOR_CORRECTOR = 0,
    SYMPLECTIC_EULER = 1
};

namespace kinova_constants
{
    //Robot ID/Name
    extern const std::string ID;

    extern const int NUMBER_OF_JOINTS;
    extern const int NUMBER_OF_SEGMENTS;
    extern const int NUMBER_OF_FRAMES;

    //Arm's root acceleration
    extern const std::vector<double> root_acceleration_1;
    extern const std::vector<double> root_acceleration_2;
    extern const std::vector<double> root_acceleration_sim;

    extern const std::vector<double> joint_position_limits_max;
    extern const std::vector<double> joint_position_limits_min;
    extern const std::vector<double> joint_position_thresholds;
    extern const std::vector<double> joint_offsets;

    extern const std::vector<double> joint_velocity_limits;
    extern const std::vector<double> joint_acceleration_limits;
    extern const std::vector<double> joint_torque_limits;
    extern const std::vector<double> joint_current_limits;
    extern const std::vector<double> joint_stopping_torque_limits;

    extern const std::vector<double> joint_inertia;
    extern const std::vector<double> joint_sim_inertia;

    extern const std::vector<double> motor_torque_constant;
    extern const std::string config_path;    
    extern const std::string urdf_path;
    extern const std::string urdf_sim_path;

    extern const std::string root_name;
    extern const std::string tooltip_name;
    extern const std::string tooltip_sim_name;
}

namespace abag_parameter
{
    // How many dimension ABAG controller is supposed to control
    extern const int DIMENSIONS;

    extern const Eigen::VectorXd ERROR_ALPHA;
    extern const Eigen::VectorXd BIAS_THRESHOLD;
    extern const Eigen::VectorXd BIAS_STEP;
    extern const Eigen::VectorXd GAIN_THRESHOLD;
    extern const Eigen::VectorXd GAIN_STEP;

    extern const double NULL_SPACE_ERROR_ALPHA;
    extern const double NULL_SPACE_BIAS_THRESHOLD;
    extern const double NULL_SPACE_BIAS_STEP;
    extern const double NULL_SPACE_GAIN_THRESHOLD;
    extern const double NULL_SPACE_GAIN_STEP;

    extern const Eigen::VectorXd MIN_BIAS_SAT_LIMIT;
    extern const Eigen::VectorXd MAX_BIAS_SAT_LIMIT;

    extern const Eigen::VectorXd MIN_GAIN_SAT_LIMIT;
    extern const Eigen::VectorXd MAX_GAIN_SAT_LIMIT;

    extern const Eigen::VectorXd MIN_COMMAND_SAT_LIMIT;
    extern const Eigen::VectorXd MAX_COMMAND_SAT_LIMIT;
}

namespace dynamics_parameter
{
    // Number of task constraints imposed on the robot, i.e. Cartesian DOFS
    extern const int NUMBER_OF_CONSTRAINTS;
    extern const int DECELERATION_UPDATE_DELAY;
    extern const int STEADY_STOP_ITERATION_THRESHOLD;
    extern const double LOWER_DECELERATION_RAMP_THRESHOLD;
    extern const double STOPPING_MOTION_LOOP_FREQ; // Hz
    extern const Eigen::VectorXd MAX_CART_FORCE;
    extern const Eigen::VectorXd MAX_CART_ACC;
    extern const Eigen::IOFormat WRITE_FORMAT;
    extern const std::string LOG_FILE_CART_PATH;
    extern const std::string LOG_FILE_STOP_MOTION_PATH;
    extern const std::string LOG_FILE_CART_BASE_PATH;
    extern const std::string LOG_FILE_JOINT_PATH;
    extern const std::string LOG_FILE_EXT_WRENCH_PATH;
    extern const std::string LOG_FILE_PREDICTIONS_PATH;
    extern const std::string LOG_FILE_NULL_SPACE_PATH;
}

namespace prediction_parameter
{
    extern const std::string CURRENT_POSE_DATA_PATH;
    extern const std::string PREDICTED_POSE_DATA_PATH;
    extern const std::string TWIST_DATA_PATH;
}

#endif /* CONSTANTS_HPP_ */