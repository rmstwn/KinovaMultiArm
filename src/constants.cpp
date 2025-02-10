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

#include <constants.hpp>

namespace kinova_constants
{
   //Robot ID/Name
   const std::string ID("kinova_gen3_lite_arm");

   // Number of joints in the manipulator
   const int NUMBER_OF_JOINTS(6);
   const int NUMBER_OF_SEGMENTS(6);
   const int NUMBER_OF_FRAMES(7);

   //Arm's root acceleration
   const std::vector<double> root_acceleration_1 {-0.07, 0.04, 9.48, 0.0, 0.0, 0.0}; // For Kinova 1
   const std::vector<double> root_acceleration_2 {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0};  // For Kinova 2
//    const std::vector<double> root_acceleration_sim {0.0, 0.0, 9.81289, 0.0, 0.0, 0.0}; // For internal simulation in the mediator
   const std::vector<double> root_acceleration_sim {-0.07, 0.04, 9.48, 0.0, 0.0, 0.0}; // For internal simulation in the mediator

   // Limits from Kinova manual-> Must be confirmed
   const std::vector<double> joint_position_limits_max {DEG_TO_RAD(9999.0), DEG_TO_RAD(127.0), DEG_TO_RAD(9999.0), DEG_TO_RAD(147.8), DEG_TO_RAD(9999.0), DEG_TO_RAD(120.3), DEG_TO_RAD(9999.0)};
   const std::vector<double> joint_position_limits_min {DEG_TO_RAD(-9999.0), DEG_TO_RAD(-127.0), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-147.8), DEG_TO_RAD(-9999.0), DEG_TO_RAD(-120.3), DEG_TO_RAD(-9999.0)};

   // Low level velocity limits (official Kinova): 2.618 rad/s (149 deg/s) for small and 1.745 rad/s (99 deg/s) for large joints
   const std::vector<double> joint_velocity_limits {1.74, 1.74, 1.74, 1.74, 2.6, 2.6, 2.6};
   const std::vector<double> joint_acceleration_limits {5.19, 5.19, 5.19, 5.19, 9.99, 9.99, 9.99};

   // High-level mode torque limits
//    const std::vector<double> joint_torque_limits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0};
   // Low-level mode torque limits: derived based on safety thresholds outlined in the KINOVA manual
   const std::vector<double> joint_torque_limits {55.0, 55.0, 55.0, 55.0, 28.0, 28.0, 28.0};

   // Low-level mode current limits: derived based on safety thresholds outlined in the KINOVA manual
   const std::vector<double> joint_current_limits {9.5, 9.5, 9.5, 9.5, 5.5, 5.5, 5.5}; // Amp

//    const std::vector<double> joint_stopping_torque_limits {0.6, 0.6, 0.6, 0.6, 0.3, 0.3, 0.3};
   const std::vector<double> joint_stopping_torque_limits {39.0, 39.0, 39.0, 39.0, 13.0, 13.0, 13.0};

   //  const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
   const std::vector<double> joint_position_thresholds {DEG_TO_RAD(0), DEG_TO_RAD(5), DEG_TO_RAD(0), DEG_TO_RAD(5), DEG_TO_RAD(0), DEG_TO_RAD(5), DEG_TO_RAD(0)};

   const std::vector<double> joint_offsets {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Rotor inertia - "d" in the algorithm:
    const std::vector<double> joint_inertia {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
   //  const std::vector<double> joint_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> joint_sim_inertia {0.5580, 0.5580, 0.5580, 0.5580, 0.1389, 0.1389, 0.1389};
   //  const std::vector<double> joint_sim_inertia {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   // Motor torque constant K_t (gear ration included - 100:1): official Kinova values
   const std::vector<double> motor_torque_constant {11.0, 11.0, 11.0, 11.0, 7.6, 7.6, 7.6}; // These ones show best result in gravity comp. cases
   // Motor torque constant K_t (gear ration included - 100:1): Kinova's manual (table with "Default threshold - warning") values
   // const std::vector<double> motor_torque_constant {5.67, 5.67, 5.67, 5.67, 4.9, 4.9, 4.9};
   // Motor torque constant K_t (gear ration included - 100:1): Kinova's manual (table with "Hard limit - upper") values
   // const std::vector<double> motor_torque_constant {8.75, 8.75, 8.75, 8.75, 6.5, 6.5, 6.5};

   // const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/kinova-gen3_urdf_V12_with_polishing_tool.urdf";
   const std::string urdf_path = "/home/rama/Documents/cpp/KinovaMultiArm/urdf/GEN3-LITE.urdf";
   const std::string urdf_sim_path = "/home/rama/Documents/cpp/KinovaMultiArm/urdf/GEN3-LITE.urdf";

   // 7 joints, 7 links, 8 frames
   const std::string root_name = "base_link";

   /**
    * With Bracelet_Link parameter, the last frame is at joint 7.
    * Mass and COM of the last (end-effector) link are included but not the real end-effector's frame.
    * Arm length: 1.12586m
    */
   const std::string tooltip_name = "Bracelet_Link";
   const std::string tooltip_sim_name = "EndEffector_Link";

   /**
    * With EndEffector_Link parameter, last frame is at the real end-effector's frame.
    * However, in the urdf model, joint between Bracelet_Link and EndEffector_Link is fixed (not counted in KDL). 
    * Vereshchagin does not support un-equal number of joints and segments
    * Arm length: 1.1873m
    */ 
//    const std::string tooltip_name = "EndEffector_Link";
}

namespace abag_parameter
{
    // How many dimension ABAG controller is supposed to control
    const int DIMENSIONS(6);

    // Error parameters: Low pass filter threshold
    const Eigen::VectorXd ERROR_ALPHA = (Eigen::VectorXd(DIMENSIONS)    << 0.800000, 0.800000, 0.900000, 0.650000, 0.850000, 0.178001).finished();

     // Bias parameters: threshold and step
    const Eigen::VectorXd BIAS_THRESHOLD = (Eigen::VectorXd(DIMENSIONS) << 0.000507, 0.000507, 0.000457, 0.001007, 0.001007, 0.724277).finished();
    const Eigen::VectorXd BIAS_STEP = (Eigen::VectorXd(DIMENSIONS)      << 0.000495, 0.000495, 0.000500, 0.003495, 0.003495, 0.503495).finished();

    // Gain parameters: threshold and step
    const Eigen::VectorXd GAIN_THRESHOLD = (Eigen::VectorXd(DIMENSIONS) << 0.452492, 0.452492, 0.500000, 0.252492, 0.252492, 0.432492).finished();
    const Eigen::VectorXd GAIN_STEP = (Eigen::VectorXd(DIMENSIONS)      << 0.002052, 0.002052, 0.002552, 0.015152, 0.015152, 0.655152).finished();

    // Parameters for controlling robot's nullspace motion
    const double NULL_SPACE_ERROR_ALPHA    = 0.900000;
    const double NULL_SPACE_BIAS_THRESHOLD = 0.000407;
    const double NULL_SPACE_BIAS_STEP      = 0.000495;
    const double NULL_SPACE_GAIN_THRESHOLD = 0.552492;
    const double NULL_SPACE_GAIN_STEP      = 0.003152;

   //  Saturation limits   
    const Eigen::VectorXd MIN_BIAS_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0).finished();
   //  const Eigen::VectorXd MIN_BIAS_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::VectorXd MAX_BIAS_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS)  << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
    const Eigen::VectorXd MIN_GAIN_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::VectorXd MAX_GAIN_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
    const Eigen::VectorXd MIN_COMMAND_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << -1.0, -1.0, -1.0, -1.0, -1.0, -1.0).finished();
   //  const Eigen::VectorXd MIN_COMMAND_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const Eigen::VectorXd MAX_COMMAND_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished();
}

namespace dynamics_parameter
{
    // Number of task constraints imposed on the robot, i.e. Cartesian DOFS
    const int NUMBER_OF_CONSTRAINTS(6);
    const int DECELERATION_UPDATE_DELAY = 5; // Iterations
    const int STEADY_STOP_ITERATION_THRESHOLD = 40; // Iterations
    const double LOWER_DECELERATION_RAMP_THRESHOLD = 0.05; // rad/sec
    const double STOPPING_MOTION_LOOP_FREQ = 750.0; // Hz  ... Higher than 750 Hz not yet feasible with the current Kinova API
    const Eigen::VectorXd MAX_CART_FORCE = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 50.0, 50.0, 200.0, 2.0, 2.0, 2.0).finished();
    const Eigen::VectorXd MAX_CART_ACC = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 100.0, 100.0, 200.0, 2.0, 2.0, 2.0).finished();
    const Eigen::IOFormat WRITE_FORMAT(6, Eigen::DontAlignCols, " ", "", "", "\n");
    const std::string LOG_FILE_CART_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/control_error.txt");
    const std::string LOG_FILE_STOP_MOTION_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/stop_motion_error.txt");
    const std::string LOG_FILE_CART_BASE_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/control_base_error.txt");
    const std::string LOG_FILE_JOINT_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/joint_torques.txt");
    const std::string LOG_FILE_EXT_WRENCH_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/ext_wrench_data.txt");
    const std::string LOG_FILE_PREDICTIONS_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/prediction_effects.txt");
    const std::string LOG_FILE_NULL_SPACE_PATH("/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/null_space_error.txt");
}

namespace prediction_parameter
{
		const std::string CURRENT_POSE_DATA_PATH = "/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/measured_pose.txt";
		const std::string PREDICTED_POSE_DATA_PATH = "/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/predicted_pose.txt";
		const std::string TWIST_DATA_PATH = "/home/rama/Documents/cpp/KinovaMultiArm/visualization/archive/current_twist.txt";
}