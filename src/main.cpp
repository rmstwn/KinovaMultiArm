/*
Author(s): Muhammad Ramadhan Hadi Setyawan
Institute: Takesue Laboratory, Tokyo Metropolitan University
Description: main.cpp

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

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <cstdint>
#include <string>
#include <iostream>
#include <future>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <stdexcept> // for std::runtime_error

// Kinova API Lib
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <InterconnectConfigClientRpc.h>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
// #include "KinovaClient/utilities.h"
// #include "KinovaClient/specifications.hpp"

// #include <kinova_manager.hpp>
#include <state_specification.hpp>
#include <fk_vereshchagin.hpp>
#include <geometry_utils.hpp>

#include <zmq.hpp>

#include "oml_mrtu.h"

#include "KinovaManager.hpp"

#define IP_ADDRESS_1 "192.168.2.10" // IP robot 1
#define IP_ADDRESS_2 "192.168.2.11" // IP robot 2
#define IP_ADDRESS_3 "192.168.2.12" // IP robot 3
#define IP_ADDRESS_4 "192.168.2.13" // IP robot 4

#define PORT 10000

// Wheel parameters
constexpr double WHEEL_CIRCUMFERENCE = 40.0 * 3.14159265359; // mm (example: 300 mm per revolution)
// constexpr double WHEEL_CIRCUMFERENCE = 400.0; // mm (example: 300 mm per revolution)

constexpr int PULSES_PER_REVOLUTION = 12000; // Encoder pulses per full wheel revolution

enum desired_pose
{
    CANDLE = 0,
    HOME = 1,
    RETRACT = 2,
    PACKAGING = 3,
    HOME_FORWARD = 4,
    HOME_BACK = 5,
    APPROACH_TABLE = 6,
    TEST = 7,
    BLANKET_1 = 10,
    BLANKET_2 = 11,
    BLANKET_3 = 12,
    BLANKET_4 = 13,
    BLANKET_5 = 14,
    BLANKET_6 = 15,
};

enum path_types
{
    SINE_PATH = 0,
    STEP_PATH = 1,
    INF_SIGN_PATH = 2
};

struct Pose
{
    double x, y, z;
    double theta_x, theta_y, theta_z;

    Pose(double px, double py, double pz, double t_x, double t_y, double t_z)
        : x(px), y(py), z(pz), theta_x(t_x), theta_y(t_y), theta_z(t_z) {}
};

// Waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration<double, std::micro> loop_interval{};

const int SECOND = 1000000;   // Number of microseconds in one second
const int MILLISECOND = 1000; // Number of microseconds in one millisecond
const int JOINTS = 6;
const int NUMBER_OF_CONSTRAINTS = 5;
int RATE_HZ = 1000; // Hz
int desired_pose_id = desired_pose::HOME;
// int desired_task_model               = task_model::full_pose;
// int desired_control_mode = control_mode::POSITION;

// Define the callback function used in Refresh_callback
auto lambda_fct_callback = [](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback data)
{
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(data.actuators(6), &serialized_data);
    std::cout << serialized_data << std::endl
              << std::endl;
};

// Function to configure motor parameters and reset alarms
bool OMconfigureMotor(ModbusAZ &motor, const std::string &motorName)
{
    if (motor.writeParamAcc(1000, 100000).empty())
    {
        std::cerr << "Failed to set acceleration parameters for " << motorName << "." << std::endl;
        return false;
    }
    if (motor.writeParamDec(1000, 100000).empty())
    {
        std::cerr << "Failed to set deceleration parameters for " << motorName << "." << std::endl;
        return false;
    }
    if (motor.writeParamCurrent(1000).empty())
    {
        std::cerr << "Failed to set current for " << motorName << "." << std::endl;
        return false;
    }

    // Reset alarm
    motor.resetAlarm();
    std::cout << "Successfully configured " << motorName << " and reset its alarm." << std::endl;
    return true;
}

// Function to move the mobile moving table
bool moveTable(ModbusAZ &motor1, ModbusAZ &motor2, double distance_mm, int speed_pulses, int operation_type)
{
    // Convert distance to encoder pulses
    int pulses = static_cast<int>((distance_mm / WHEEL_CIRCUMFERENCE) * PULSES_PER_REVOLUTION);

    std::cout << "Moving table " << distance_mm << " mm -> " << pulses << " pulses" << std::endl;

    // Move motor 1
    if (motor1.startPosition(pulses, speed_pulses, operation_type).empty())
    {
        std::cerr << "Failed to move Motor 1." << std::endl;
        return false;
    }

    // Move motor 2 in reverse direction (assuming differential drive)
    if (motor2.startPosition(pulses, speed_pulses, operation_type).empty())
    {
        std::cerr << "Failed to move Motor 2." << std::endl;
        return false;
    }

    while (true)
    {
        std::vector<int> positionVec1 = motor1.readPosition();
        std::vector<int> positionVec2 = motor2.readPosition();
        if (positionVec1.empty() || positionVec2.empty())
        {
            std::cerr << "❌ Failed to read position for motor(s) during movement." << std::endl;
            return false;
        }

        int currentPosition1 = positionVec1[1];
        int currentPosition2 = positionVec2[1];

        std::cout << "pulses: " << pulses << std::endl;

        std::cout << "Position motor 1: " << currentPosition1 << std::endl;
        std::cout << "Position motor 2: " << currentPosition1 << std::endl;

        if (currentPosition1 == pulses && currentPosition2 == pulses)
        {
            std::cout << "✅ Target position reached for motors 1 and 2." << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return true;
}

// Function to rotate the mobile table
bool rotateTable(ModbusAZ &motor3, double angle_degrees, int speed_pulses, int operation_type)
{
    // Convert angle to encoder pulses
    // 9000 pulses = 90 degrees, so we calculate the pulses per degree
    int pulses_per_degree = 9000 / 90;
    int pulses = static_cast<int>(angle_degrees * pulses_per_degree);

    std::cout << "Rotating table " << angle_degrees << " degrees -> " << pulses << " pulses" << std::endl;

    // Move motor 3 to rotate the table
    if (motor3.startPosition(pulses, speed_pulses, operation_type).empty())
    {
        std::cerr << "Failed to rotate Motor 3." << std::endl;
        return false;
    }

    return true;
}

// // Function to move and rotate the table to a specific position (or home position)
// bool goToTable(ModbusAZ &motor1, ModbusAZ &motor2, ModbusAZ &motor3, double distance_mm, double angle_degrees, int speed_pulses, int operation_type)
// {
//     // Move the table to the desired position (distance)
//     if (!moveTable(motor1, motor2, distance_mm, speed_pulses, operation_type))
//     {
//         std::cerr << "Failed to move the table." << std::endl;
//         return false;
//     }

//     // Rotate the table to the desired angle
//     if (!rotateTable(motor3, angle_degrees, speed_pulses, operation_type))
//     {
//         std::cerr << "Failed to rotate the table." << std::endl;
//         return false;
//     }

//     std::cout << "Table moved and rotated successfully." << std::endl;
//     return true;
// }

// // Function to move and rotate the table to a specific position
// void goToTable(ModbusAZ &motor1, ModbusAZ &motor2, double distance_mm, int linear_speed_pulses, ModbusAZ &motor3,
//                 double angle_degrees, int rotate_speed_pulses, int operation_type)
// {
//     std::cout << "Moving table: Distance = " << distance_mm << " mm, Angle = " << angle_degrees << " degrees" << std::endl;

//     // Move the table to the specified distance
//     if (!moveTable(motor1, motor2, distance_mm, linear_speed_pulses, operation_type))
//     {
//         std::cerr << "Error: Failed to move the table to " << distance_mm << " mm." << std::endl;
//         return;
//     }

//     std::this_thread::sleep_for(std::chrono::milliseconds(200));

//     // Rotate the table to the specified angle
//     if (!rotateTable(motor3, angle_degrees, rotate_speed_pulses, operation_type))
//     {
//         std::cerr << "Error: Failed to rotate the table to " << angle_degrees << " degrees." << std::endl;
//         return;
//     }

//     std::cout << "✅ Table movement complete: " << distance_mm << " mm, " << angle_degrees << " degrees" << std::endl;
// }

// void goToTable(ModbusAZ &motor1, ModbusAZ &motor2, double distance_mm, int linear_speed_pulses,
//                ModbusAZ &motor3, double angle_degrees, int rotate_speed_pulses, int operation_type)
// {
//     std::cout << "🟢 Moving table: Distance = " << distance_mm << " mm, Angle = " << angle_degrees << " degrees" << std::endl;

//     // Move the table
//     std::cout << "➡ Moving table..." << std::endl;
//     bool moveSuccess = moveTable(motor1, motor2, distance_mm, linear_speed_pulses, operation_type);
//     std::cout << "✅ Move result: " << moveSuccess << std::endl;
//     if (!moveSuccess)
//     {
//         std::cerr << "❌ Error: Failed to move the table to " << distance_mm << " mm." << std::endl;
//         return;
//     }

//     std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Increased delay

//     // Rotate the table
//     std::cout << "🔄 Rotating table..." << std::endl;
//     bool rotateSuccess = rotateTable(motor3, angle_degrees, rotate_speed_pulses, operation_type);
//     std::cout << "✅ Rotate result: " << rotateSuccess << std::endl;
//     if (!rotateSuccess)
//     {
//         std::cerr << "❌ Error: Failed to rotate the table to " << angle_degrees << " degrees." << std::endl;
//         return;
//     }

//     std::cout << "🏁 Table movement complete: " << distance_mm << " mm, " << angle_degrees << " degrees" << std::endl;
// }

void goToTable(ModbusAZ &motor1, ModbusAZ &motor2, double distance_mm, int linear_speed_pulses,
               ModbusAZ &motor3, double angle_degrees, int rotate_speed_pulses, int operation_type)
{

    // Rotate the table
    std::cout << "🔄 Rotating table..." << std::endl;
    bool rotateSuccess = rotateTable(motor3, angle_degrees, rotate_speed_pulses, operation_type);
    std::cout << "✅ Rotate result: " << rotateSuccess << std::endl;
    if (!rotateSuccess)
    {
        std::cerr << "❌ Error: Failed to rotate the table to " << angle_degrees << " degrees." << std::endl;
        return;
    }

    // Wait for a bit to ensure the move is completed
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Increased delay

    std::cout << "🟢 Moving table: Distance = " << distance_mm << " mm, Angle = " << angle_degrees << " degrees" << std::endl;

    // Move the table
    std::cout << "➡ Moving table..." << std::endl;
    bool moveSuccess = moveTable(motor1, motor2, distance_mm, linear_speed_pulses, operation_type);
    std::cout << "✅ Move result: " << moveSuccess << std::endl;
    if (!moveSuccess)
    {
        std::cerr << "❌ Error: Failed to move the table to " << distance_mm << " mm." << std::endl;
        return;
    }

    // // Wait for the position feedback to match the target
    // std::cout << "⏳ Waiting for the position to reach the target..." << std::endl;
    // while (true)
    // {
    //     std::vector<int> positionVec1 = motor1.readPosition();
    //     std::vector<int> positionVec2 = motor2.readPosition();
    //     if (positionVec1.empty() || positionVec2.empty())
    //     {
    //         std::cerr << "❌ Failed to read position for motor(s) during movement." << std::endl;
    //         return;
    //     }

    //     int currentPosition1 = positionVec1[1]; // Assume index 0 has the position
    //     int currentPosition2 = positionVec2[1]; // Assume index 0 has the position

    //     // Check if the current positions match the target positions (pulse-based)
    //     if (currentPosition1 == linear_speed_pulses && currentPosition2 == linear_speed_pulses)
    //     {
    //         std::cout << "✅ Target position reached for motors 1 and 2." << std::endl;
    //         break;
    //     }

    //     // Optionally add a timeout or break after a certain number of iterations
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust wait time if needed
    // }

    // // Wait for the position feedback for motor 3 to match target
    // std::cout << "⏳ Waiting for motor 3 to reach target position..." << std::endl;
    // while (true)
    // {
    //     std::vector<int> positionVec3 = motor3.readPosition();
    //     if (positionVec3.empty())
    //     {
    //         std::cerr << "❌ Failed to read position for motor 3." << std::endl;
    //         return;
    //     }

    //     int currentPosition3 = positionVec3[1]; // Assume index 0 has the position

    //     // Check if the current position matches the target position (pulse-based)
    //     if (currentPosition3 == rotate_speed_pulses)
    //     {
    //         std::cout << "✅ Target position reached for motor 3." << std::endl;
    //         break;
    //     }

    //     // Optionally add a timeout or break after a certain number of iterations
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Adjust wait time if needed
    // }

    std::cout << "🏁 Table movement complete: " << distance_mm << " mm, " << angle_degrees << " degrees" << std::endl;
}

// Shared variables for vision-based coordinates
std::mutex coord_mutex;
double vision_x_cam = 0.0, vision_y_cam = 0.0, vision_z_cam = 0.0;
double vision_x_base_r = 0.0, vision_y_base_r = 0.0, vision_z_base_r = 0.0;
double vision_x_base_l = 0.0, vision_y_base_l = 0.0, vision_z_base_l = 0.0;

// Function to receive vision coordinates from the ZMQ topic "P_camera"
void visionReceiverCamera()
{
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_SUB);
    socket.connect("tcp://192.168.0.10:5555");
    std::string topic = "P_camera";
    socket.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    while (true)
    {
        zmq::message_t zmq_message;
        socket.recv(zmq_message, zmq::recv_flags::none);
        std::string message(static_cast<char *>(zmq_message.data()) + topic.size(), zmq_message.size());

        std::lock_guard<std::mutex> lock(coord_mutex);
        sscanf(message.c_str(), "%lf %lf %lf", &vision_x_cam, &vision_y_cam, &vision_z_cam);

        std::cout << "Received vision coordinates P_camera: X=" << vision_x_cam << " Y=" << vision_y_cam << " Z=" << vision_z_cam << std::endl;
    }
}

// Function to receive vision coordinates from the ZMQ topic "P_base_r"
void visionReceiverBaseR()
{
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_SUB);
    socket.connect("tcp://192.168.0.10:5555");
    std::string topic = "P_base_r";
    socket.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    while (true)
    {
        zmq::message_t zmq_message;
        socket.recv(zmq_message, zmq::recv_flags::none);
        std::string message(static_cast<char *>(zmq_message.data()) + topic.size(), zmq_message.size());

        std::lock_guard<std::mutex> lock(coord_mutex);
        sscanf(message.c_str(), "%lf %lf %lf", &vision_x_base_r, &vision_y_base_r, &vision_z_base_r);

        std::cout << "Received vision coordinates P_base_r: X=" << vision_x_base_r << " Y=" << vision_y_base_r << " Z=" << vision_z_base_r << std::endl;
    }
}

// Function to receive vision coordinates from the ZMQ topic "P_base_l"
void visionReceiverBaseL()
{
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_SUB);
    socket.connect("tcp://192.168.0.10:5555");
    std::string topic = "P_base_l";
    socket.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    while (true)
    {
        zmq::message_t zmq_message;
        socket.recv(zmq_message, zmq::recv_flags::none);
        std::string message(static_cast<char *>(zmq_message.data()) + topic.size(), zmq_message.size());

        std::lock_guard<std::mutex> lock(coord_mutex);
        sscanf(message.c_str(), "%lf %lf %lf", &vision_x_base_l, &vision_y_base_l, &vision_z_base_l);

        std::cout << "Received vision coordinates P_base_l: X=" << vision_x_base_l << " Y=" << vision_y_base_l << " Z=" << vision_z_base_l << std::endl;
    }
}

// void move_robot(KinovaManager &robot, int pose)
// {
//     robot.go_to(pose); // Use go_to or go_to_cart as needed
// }

// int main(int argc, char **argv)
// {
//     // // Start threads to receive data from each topic
//     // std::thread camera_thread(visionReceiverCamera);
//     // std::thread base_r_thread(visionReceiverBaseR);
//     // std::thread base_l_thread(visionReceiverBaseL);

//     // int return_flag = 0;

//     //////////////////////////////////////// START Kinova Initialization ////////////////////////////////////////////
//     // // Kinova Robot Initialization
//     // desired_pose_id = desired_pose::HOME;
//     // desired_control_mode = control_mode::POSITION;

//     // kinova_manager robot_driver_1, robot_driver_2, robot_driver_3, robot_driver_4;

//     // if (!robot_driver_1.is_initialized())
//     //     robot_driver_1.initialize(robot_id::KINOVA_GEN3_lITE_1, 1.0 / static_cast<double>(RATE_HZ));
//     // if (!robot_driver_1.is_initialized())
//     // {
//     //     std::cerr << "Robot 1 not initialized.\n";
//     //     return 0;
//     // }

//     // if (!robot_driver_2.is_initialized())
//     //     robot_driver_2.initialize(robot_id::KINOVA_GEN3_lITE_2, 1.0 / static_cast<double>(RATE_HZ));
//     // if (!robot_driver_2.is_initialized())
//     // {
//     //     std::cerr << "Robot 2 not initialized.\n";
//     //     return 0;
//     // }

//     // if (!robot_driver_3.is_initialized())
//     //     robot_driver_3.initialize(robot_id::KINOVA_GEN3_lITE_3, 1.0 / static_cast<double>(RATE_HZ));
//     // if (!robot_driver_3.is_initialized())
//     // {
//     //     std::cerr << "Robot 3 not initialized.\n";
//     //     return 0;
//     // }

//     // if (!robot_driver_4.is_initialized())
//     //     robot_driver_4.initialize(robot_id::KINOVA_GEN3_lITE_4, 1.0 / static_cast<double>(RATE_HZ));
//     // if (!robot_driver_4.is_initialized())
//     // {
//     //     std::cerr << "Robot 4 not initialized.\n";
//     //     return 0;
//     // }

//     // Initialize robot managers
//     KinovaManager robot_1(IP_ADDRESS_1, 10000);
//     // KinovaManager robot_2(IP_ADDRESS_2, 10000);
//     // KinovaManager robot_3(IP_ADDRESS_3, 10000);
//     // KinovaManager robot_4(IP_ADDRESS_4, 10000);

//     //////////////////////////////////////// END Kinova Initialization ////////////////////////////////////////////
//     //////////////////////////////////////// START Moving tables Initialization ////////////////////////////////////////////
//     // Mobile Moving Table Initialization

//     // // First Moving Table Initialization
//     // CommPC commport1("/dev/ttyUSB0", 115200);
//     // ModbusAZ motor1(&commport1, 1), motor2(&commport1, 2), motor3(&commport1, 3);

//     // if (!OMconfigureMotor(motor1, "Motor 1") || !OMconfigureMotor(motor2, "Motor 2") || !OMconfigureMotor(motor3, "Motor 3"))
//     //     return 0;

//     // // Move the first table to the home position (0 mm distance, 0 degrees)
//     // if (!goToTable(motor1, motor2, motor3, 0.0, 0.0, 2000, 1))
//     // {
//     //     std::cerr << "Failed to move the first table to home position." << std::endl;
//     //     return 0;
//     // }

//     // // Second Moving Table Initialization
//     // CommPC commport2("/dev/ttyUSB1", 115200); // Assuming the second table uses a different port
//     // ModbusAZ motor4(&commport2, 4), motor5(&commport2, 5), motor6(&commport2, 6);

//     // if (!OMconfigureMotor(motor4, "Motor 4") || !OMconfigureMotor(motor5, "Motor 5") || !OMconfigureMotor(motor6, "Motor 6"))
//     //     return 0;

//     // // Move the second table to the home position (0 mm distance, 0 degrees)
//     // if (!goToTable(motor4, motor5, motor6, 0.0, 0.0, 2000, 1))
//     // {
//     //     std::cerr << "Failed to move the second table to home position." << std::endl;
//     //     return 0;
//     // }

//     //////////////////////////////////////// END Moving tables Initialization ////////////////////////////////////////////

//     // gripper_with_error_handling(robot_driver_1, 0.01, robot_driver_2, 0.01, 1000); // open 1 2 gripper

//     // // Wait for a vision update before moving the robot
//     // std::this_thread::sleep_for(std::chrono::seconds(2));

//     // {
//     //     std::lock_guard<std::mutex> lock(coord_mutex);
//     //     std::cout << "Using vision-based coordinates: X=" << vision_x << " Y=" << vision_y << " Z=" << vision_z << std::endl;
//     // }

//     // // Robot Task Execution using received coordinates
//     // go_to_cart_with_error_handling(robot_driver_1, 0.5, 10, robot_driver_2, 0.5, 10, desired_pose::BLANKET_1);

//     // // Deinitialize robots
//     // robot_driver_1.deinitialize();
//     // robot_driver_2.deinitialize();

//     // std::cout << "Kinova Multi-Arm and Mobile Table Execution Completed!" << std::endl;

//     // robot_1.go_to(desired_pose::CANDLE);  // CANDLE pose

//     // // Thread sequence for moving to multiple poses
//     // std::thread robot_thread_1(move_robot, std::ref(robot_1), desired_pose::CANDLE); // Move to CANDLE pose
//     // robot_thread_1.join();                                                           // Wait for the thread to finish before moving to the next pose

//     // std::thread robot_thread_2(move_robot, std::ref(robot_1), desired_pose::HOME); // Move to BLANKET_1 pose
//     // robot_thread_2.join();                                                              // Wait for the thread to finish before moving to the next pose

//     // std::thread robot_thread_3(move_robot, std::ref(robot_1), desired_pose::CANDLE); // Move to HOME pose
//     // robot_thread_3.join();                                                         // Wait for the thread to finish before moving to the next pose

//     robot_1.stopConnection();

//     // // Join the threads before exiting the program
//     // camera_thread.join();
//     // base_r_thread.join();
//     // base_l_thread.join();

//     return 0;
// }

// Function to move robot to the desired pose
void move_robot(KinovaManager &robot, desired_pose pose, std::mutex &robot_mutex)
{
    try
    {
        std::lock_guard<std::mutex> lock(robot_mutex); // Lock the mutex to ensure thread safety

        // Initialize robot connection if not already connected
        if (!robot.is_connected())
        {
            robot.setupConnection(); // Initialize connection
            if (!robot.is_connected())
            { // Check again after setup
                throw std::runtime_error("Failed to initialize the robot.");
            }
            std::cout << "Robot initialized successfully.\n";
        }

        // // Move robot to the desired pose
        // if (!robot.go_to(pose)) {
        //     throw std::runtime_error("Failed to move robot to desired pose.");
        // }

        robot.go_to(pose);
        // robot.go_to_cart(0.5, 10, pose);

        std::cout << "Robot moved to pose: " << static_cast<int>(pose) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

// Gripper control function using KinovaManager::gripper
void control_gripper(KinovaManager &robot, float position, int64_t time)
{
    try
    {
        robot.gripper(position, time); // Set the gripper position with a duration
        std::cout << "Gripper position set to " << position << " with time " << time << " ms." << std::endl;
    }
    catch (const std::exception &ex)
    {
        std::cout << "Error controlling gripper: " << ex.what() << std::endl;
    }
}

void executeSequence(const std::vector<std::function<void()>> &tasks)
{
    std::vector<std::thread> threads;

    // Launch all tasks in parallel
    for (const auto &task : tasks)
    {
        threads.emplace_back(task);
    }

    // Wait for all tasks to complete
    for (auto &thread : threads)
    {
        thread.join();
    }
}

int main()
{
    ////////////////////////////////////// START Moving tables Initialization ////////////////////////////////////////////
    // Mobile Moving Table Initialization

    // First Moving Table Initialization
    CommPC commport1("/dev/ttyUSB0", 115200);
    ModbusAZ motor1(&commport1, 1), motor2(&commport1, 2), motor3(&commport1, 3);

    if (!OMconfigureMotor(motor1, "Motor 1") || !OMconfigureMotor(motor2, "Motor 2") || !OMconfigureMotor(motor3, "Motor 3"))
        return 0;

    ////////////////////////////////////// END Moving tables Initialization ////////////////////////////////////////////
    //////////////////////////////////////// START Kinova Initialization ////////////////////////////////////////////
    // Kinova Robot Initialization
    desired_pose_id = desired_pose::HOME;

    // Initialize robot managers
    KinovaManager kinova_1(IP_ADDRESS_1, 10000);
    KinovaManager kinova_2(IP_ADDRESS_2, 10000);
    // KinovaManager robot_3(IP_ADDRESS_3, 10000);
    // KinovaManager robot_4(IP_ADDRESS_4, 10000);

    //////////////////////////////////////// END Kinova Initialization ////////////////////////////////////////////

    std::mutex kinova_mutex_1, kinova_mutex_2, kinova_mutex_3, kinova_mutex_4;

    // // Move the first table to the home position (0 mm distance, 0 degrees)
    // std::thread table_thread_1([&]()
    //                            { goToTable(motor1, motor2, 0.0, 5000, motor3, 0.0, 2000, 1); });

    // std::thread kinova_thread_1([&]()
    //                             { move_robot(kinova_1, PACKAGING, kinova_mutex_1); });
    // std::thread kinova_thread_2([&]()
    //                             { move_robot(kinova_2, PACKAGING, kinova_mutex_2); });

    // // Optionally wait for the thread to finish execution
    // table_thread_1.join();
    // kinova_thread_1.join();
    // kinova_thread_2.join();

    // /////////////////////////////////////////////////////////////////////////////////////////////////////

    // // Move the first table to the next position (500 mm distance, 0 degrees)
    // std::thread table_thread_2([&]()
    //                            { goToTable(motor1, motor2, 500.0, 5000, motor3, 0.0, 2000, 1); });

    // std::thread kinova_thread_3([&]()
    //                             { move_robot(kinova_1, HOME, kinova_mutex_1); });
    // std::thread kinova_thread_4([&]()
    //                             { move_robot(kinova_2, HOME, kinova_mutex_2); });

    // // Optionally wait for the thread to finish execution
    // table_thread_2.join();
    // kinova_thread_3.join();
    // kinova_thread_4.join();

    // // Define movement sequences
    // std::vector<std::vector<std::function<void()>>> sequences = {
    //     {[&]()
    //      { control_gripper(kinova_1, 0.01, 10); },
    //      [&]()
    //      { control_gripper(kinova_2, 0.01, 10); },
    //      [&]()
    //      { goToTable(motor1, motor2, 0.0, 5000, motor3, 0.0, 2000, 1); },
    //      [&]()
    //      { move_robot(kinova_1, PACKAGING, kinova_mutex_1); },
    //      [&]()
    //      { move_robot(kinova_2, PACKAGING, kinova_mutex_2); }},
    //     {[&]()
    //      { goToTable(motor1, motor2, 500.0, 5000, motor3, 0.0, 2000, 1); },
    //      [&]()
    //      { move_robot(kinova_1, HOME, kinova_mutex_1); },
    //      [&]()
    //      { move_robot(kinova_2, HOME, kinova_mutex_2); }}};

    // Define movement sequences
    std::vector<std::vector<std::function<void()>>> sequences = {
        {[&]()
         { control_gripper(kinova_1, 0.01, 10); },
         [&]()
         { control_gripper(kinova_2, 0.01, 10); }},

        {[&]()
         { goToTable(motor1, motor2, 0.0, 5000, motor3, 0.0, 2000, 1); },
         [&]()
         { move_robot(kinova_1, PACKAGING, kinova_mutex_1); },
         [&]()
         { move_robot(kinova_2, PACKAGING, kinova_mutex_2); }},

        // {[&]()
        //  { goToTable(motor1, motor2, 500.0, 5000, motor3, 0.0, 2000, 1); },
        //  [&]()
        //  { move_robot(kinova_1, HOME, kinova_mutex_1); },
        //  [&]()
        //  { move_robot(kinova_2, HOME, kinova_mutex_2); }},

        // {[&]()
        //  { goToTable(motor1, motor2, 500.0, 5000, motor3, 20.0, 2000, 1); },
        //  [&]()
        //  { control_gripper(kinova_1, 0.9, 10); },
        //  [&]()
        //  { control_gripper(kinova_2, 0.9, 10); }},

    };

    // Execute each sequence step by step
    for (const auto &sequence : sequences)
    {
        executeSequence(sequence);
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // goToTable(motor1, motor2, 0.0, 4000, motor3, 0.0, 2000, 1);

    // goToTable(motor1, motor2, 1000.0, 4000, motor3, 0.0, 2000, 1);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // goToTable(motor1, motor2, 0.0, 4000, motor3, 0.0, 2000, 1);
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // if (!rotateTable(motor3, 45.0, 2000, 1))
    // {
    //     std::cerr << "Failed to rotate the table." << std::endl;
    //     return 0;
    // }

    // // Second Moving Table Initialization
    // CommPC commport2("/dev/ttyUSB1", 115200); // Assuming the second table uses a different port
    // ModbusAZ motor4(&commport2, 4), motor5(&commport2, 5), motor6(&commport2, 6);

    // if (!OMconfigureMotor(motor4, "Motor 4") || !OMconfigureMotor(motor5, "Motor 5") || !OMconfigureMotor(motor6, "Motor 6"))
    //     return 0;

    // // Move the second table to the home position (0 mm distance, 0 degrees)
    // goToTable(motor4, motor5, motor6, 0.0, 0.0, 2000, 1);

    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ////////////////////////////////////// END Moving tables Initialization ////////////////////////////////////////////

    // // Initialize robot managers for 4 robots
    // KinovaManager robot_1(IP_ADDRESS_1, 10000);
    // KinovaManager robot_2(IP_ADDRESS_2, 10000);
    // // KinovaManager robot_3(IP_ADDRESS_3, 10000);
    // // KinovaManager robot_4(IP_ADDRESS_4, 10000);

    // // Create threads to move tables simultaneously
    // std::thread table_thread_1(goToTable, std::ref(motor1), std::ref(motor2), std::ref(motor3),
    //                            0.0, 0.0, 2000, 1);

    // // Wait for moving table threads to finish
    // table_thread_1.join();

    // // Mutexes for thread safety
    // std::mutex robot_mutex_1, robot_mutex_2, robot_mutex_3, robot_mutex_4;

    // // Create threads for opening the grippers first
    // std::thread gripper_thread_1([&]()
    //                              { control_gripper(robot_1, 0.01, 10); }); // Open gripper for robot_1
    // std::thread gripper_thread_2([&]()
    //                              { control_gripper(robot_2, 0.01, 10); }); // Open gripper for robot_2
    // // std::thread gripper_thread_3([&]() { control_gripper(robot_3, 0.01, 10); }); // Open gripper for robot_3
    // // std::thread gripper_thread_4([&]() { control_gripper(robot_4, 0.01, 10); }); // Open gripper for robot_4

    // // Wait for gripper threads to finish opening the grippers
    // gripper_thread_1.join();
    // gripper_thread_2.join();
    // // gripper_thread_3.join();
    // // gripper_thread_4.join();

    // // Create threads for moving robots to CANDLE pose after opening grippers
    // std::thread robot_thread_1([&]()
    //                            { move_robot(robot_1, HOME, robot_mutex_1); });
    // std::thread robot_thread_2([&]()
    //                            { move_robot(robot_2, CANDLE, robot_mutex_2); });
    // // std::thread robot_thread_3([&]() { move_robot(robot_3, CANDLE, robot_mutex_3); });
    // // std::thread robot_thread_4([&]() { move_robot(robot_4, CANDLE, robot_mutex_4); });

    // // Wait for robots to finish moving to CANDLE pose
    // robot_thread_1.join();
    // robot_thread_2.join();
    // // robot_thread_3.join();
    // // robot_thread_4.join();

    // // Create threads for closing the grippers after reaching CANDLE pose
    // std::thread gripper_thread_5([&]()
    //                              { control_gripper(robot_1, 0.9, 10); }); // Close gripper for robot_1
    // std::thread gripper_thread_6([&]()
    //                              { control_gripper(robot_2, 0.9, 10); }); // Close gripper for robot_2
    // // std::thread gripper_thread_7([&]() { control_gripper(robot_3, 1.0, 10); }); // Close gripper for robot_3
    // // std::thread gripper_thread_8([&]() { control_gripper(robot_4, 1.0, 10); }); // Close gripper for robot_4

    // // Wait for gripper threads to finish closing the grippers
    // gripper_thread_5.join();
    // gripper_thread_6.join();
    // // gripper_thread_7.join();
    // // gripper_thread_8.join();

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // // Move all robots to HOME pose
    // std::thread robot_thread_5([&]()
    //                            { move_robot(robot_1, CANDLE, robot_mutex_1); });
    // std::thread robot_thread_6([&]()
    //                            { move_robot(robot_2, HOME, robot_mutex_2); });
    // // std::thread robot_thread_7([&]() { move_robot(robot_3, HOME, robot_mutex_3); });
    // // std::thread robot_thread_8([&]() { move_robot(robot_4, HOME, robot_mutex_4); });

    // // Wait for all robots to finish moving to HOME pose
    // robot_thread_5.join();
    // robot_thread_6.join();
    // // robot_thread_7.join();
    // // robot_thread_8.join();

    // // Stop the connection with all robots
    // robot_1.stopConnection();
    // robot_2.stopConnection();
    // // robot_3.stopConnection();
    // // robot_4.stopConnection();

    return 0;
}

// #include <iostream>
// #include <thread>
// #include "KinovaManager.hpp"

// void move_robot(KinovaManager& robot, int pose) {
//     robot.go_to(pose);  // Use go_to or go_to_cart as needed
// }

// int main() {
//     // IP addresses of the robots
//     std::string ip_address_1 = "192.168.1.1";
//     std::string ip_address_2 = "192.168.1.2";
//     std::string ip_address_3 = "192.168.1.3";
//     std::string ip_address_4 = "192.168.1.4";

//     // Initialize robot managers
//     KinovaManager robot_1(ip_address_1, 10000);
//     KinovaManager robot_2(ip_address_2, 10000);
//     KinovaManager robot_3(ip_address_3, 10000);
//     KinovaManager robot_4(ip_address_4, 10000);

//     // Create threads to move robots simultaneously
//     std::thread robot_thread_1(move_robot, std::ref(robot_1), 1);  // CANDLE pose
//     std::thread robot_thread_2(move_robot, std::ref(robot_2), 2);  // APPROACH_TABLE pose
//     std::thread robot_thread_3(move_robot, std::ref(robot_3), 3);  // HOME pose
//     std::thread robot_thread_4(move_robot, std::ref(robot_4), 4);  // CANDLE pose

//     // Wait for all robots to finish moving
//     robot_thread_1.join();
//     robot_thread_2.join();
//     robot_thread_3.join();
//     robot_thread_4.join();

//     return 0;
// }

// int main(int argc, char **argv)
// {
//     RATE_HZ = 700; // Hz

//     desired_pose_id = desired_pose::HOME;
//     desired_control_mode = control_mode::POSITION;

//     kinova_manager robot_driver_1;
//     kinova_manager robot_driver_2;

//     int return_flag = 0;

//     // Extract robot model and if not simulation, establish connection with motor drivers
//     if (!robot_driver_1.is_initialized())
//         robot_driver_1.initialize(robot_id::KINOVA_GEN3_lITE_1, 1.0 / static_cast<double>(RATE_HZ));
//     if (!robot_driver_1.is_initialized())
//     {
//         printf("Robot 1 is not initialized\n");
//         return 0;
//     }

//     if (!robot_driver_2.is_initialized())
//         robot_driver_2.initialize(robot_id::KINOVA_GEN3_lITE_2, 1.0 / static_cast<double>(RATE_HZ));
//     if (!robot_driver_2.is_initialized())
//     {
//         printf("Robot 2 is not initialized\n");
//         return 0;
//     }

//     // return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
//     // if (return_flag != 0)
//     //     return 0;

//     // return_flag = go_to_cart(robot_driver_1, 0.5, 10, robot_driver_2, 0.1, 10, desired_pose::BLANKET_1);
//     // if (return_flag != 0)
//     //     return 0;

//     // return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
//     // if (return_flag != 0)
//     //     return 0;

//     // gripper_with_ekrror_handling(robot_driver_1, 0.01, robot_driver_2, 0.01, 1000); // open 1 2 gripper

//     // go_to_with_error_handling(robot_driver_1, robot_driver_2, desired_pose::HOME); // go to Home

//     // go_to_cart_with_error_handling(robot_driver_1, 0.5, 10, robot_driver_2, 0.1, 10, desired_pose::BLANKET_1);

//     // gripper_with_error_handling(robot_driver_1, 0.01, robot_driver_2, 0.9, 10); // close 2 gripper

//     // go_to_with_error_handling(robot_driver_1, robot_driver_2, desired_pose::HOME);

//     go_to_cart_with_error_handling(robot_driver_1, 0.5, 10, robot_driver_2, 0.5, 10, desired_pose::BLANKET_2);

//     // gripper_with_error_handling(robot_driver_1, 0.01, robot_driver_2, 0.9, 10); // close 2 gripper

//     // go_to_cart_with_error_handling(robot_driver_1, 0.5, 10, robot_driver_2, 0.5, 10, desired_pose::BLANKET_1);

//     // gripper_with_error_handling(robot_driver_1, 0.01, robot_driver_2, 0.01, 1000); // open 1 2 gripper

//     robot_driver_1.deinitialize();
//     robot_driver_2.deinitialize();

//     std::cout << "Kinova Multi-Arm Project Finish!" << std::endl;
//     return 0;
// }