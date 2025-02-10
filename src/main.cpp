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

#include <kinova_manager.hpp>
#include <state_specification.hpp>
#include <fk_vereshchagin.hpp>
#include <geometry_utils.hpp>

#define IP_ADDRESS_1 "192.168.2.10" // IP robot 1
#define IP_ADDRESS_2 "192.168.2.11" // IP robot 2
#define PORT 10000

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
int desired_control_mode = control_mode::POSITION;

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(Kinova::Api::Base::ActionNotification)> create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent> &finish_promise)
{
    return [&finish_promise](Kinova::Api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case Kinova::Api::Base::ActionEvent::ACTION_END:
        case Kinova::Api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

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

int go_to(kinova_manager &robot_driver_1, kinova_manager &robot_driver_2, const int desired_pose_)
{
    std::vector<double> configuration_array(6, 0.0);
    switch (desired_pose_) // Angle value are in units of degree
    {
    case desired_pose::CANDLE:
        configuration_array = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        break;
    case desired_pose::APPROACH_TABLE:
        configuration_array = std::vector<double>{0.001, 42.017, 179.56, 220.641, 2.761, 1.965};
        break;
    case desired_pose::HOME_BACK:
        configuration_array = std::vector<double>{356.129, 304.126, 181.482, 250.087, 2.852, 328.367};
        break;
    case desired_pose::PACKAGING:
        configuration_array = std::vector<double>{270.0, 148.0, 148.0, 270.0, 140.0, 0.0};
        break;
    case desired_pose::RETRACT:
        configuration_array = std::vector<double>{0.0, 340.0, 180.0, 214.0, 0.0, 310.0};
        break;
    case desired_pose::HOME:
        configuration_array = std::vector<double>{0.0, 344.0, 75.0, 0.0, 300.0, 0.0};
        break;
    default:
        return -1;
    }

    auto error_callback = [](Kinova::Api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    //////////////////////////////////////////////////////////////////////
    // Kinova 1 Create API objects
    auto transport_1 = new Kinova::Api::TransportClientTcp();
    auto router_1 = new Kinova::Api::RouterClient(transport_1, error_callback);
    transport_1->connect(IP_ADDRESS_1, PORT);
    // Set session data connection information
    auto create_session_info_1 = Kinova::Api::Session::CreateSessionInfo();
    create_session_info_1.set_username("admin");
    create_session_info_1.set_password("admin");
    create_session_info_1.set_session_inactivity_timeout(200);    // (milliseconds)
    create_session_info_1.set_connection_inactivity_timeout(200); // (milliseconds)
    // Session manager service wrapper
    auto session_manager_1 = new Kinova::Api::SessionManager(router_1);
    session_manager_1->CreateSession(create_session_info_1);
    // Create services
    auto base_1 = new Kinova::Api::Base::BaseClient(router_1);
    //////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////
    // Kinova 2 Create API objects
    auto transport_2 = new Kinova::Api::TransportClientTcp();
    auto router_2 = new Kinova::Api::RouterClient(transport_2, error_callback);
    transport_2->connect(IP_ADDRESS_2, PORT);

    // Set session data connection information
    auto create_session_info_2 = Kinova::Api::Session::CreateSessionInfo();
    create_session_info_2.set_username("admin");
    create_session_info_2.set_password("admin");
    create_session_info_2.set_session_inactivity_timeout(200);    // (milliseconds)
    create_session_info_2.set_connection_inactivity_timeout(200); // (milliseconds)
    // Session manager service wrapper
    auto session_manager_2 = new Kinova::Api::SessionManager(router_2);
    session_manager_2->CreateSession(create_session_info_2);
    // Create services
    auto base_2 = new Kinova::Api::Base::BaseClient(router_2);
    //////////////////////////////////////////////////////////////////////

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_1->SetServoingMode(servoingMode);
    base_2->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto constrained_joint_angles = Kinova::Api::Base::ConstrainedJointAngles();
    auto joint_angles = constrained_joint_angles.mutable_joint_angles();
    auto actuator_count = base_1->GetActuatorCount();

    for (size_t i = 0; i < actuator_count.count(); ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(configuration_array[i]);
    }

    // Connect to notification action topic (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<Kinova::Api::Base::ActionEvent> finish_promise_1;
    auto finish_future_1 = finish_promise_1.get_future();
    auto promise_notification_handle_1 = base_1->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise_1),
        Kinova::Api::Common::NotificationOptions());
    std::promise<Kinova::Api::Base::ActionEvent> finish_promise_2;
    auto finish_future_2 = finish_promise_2.get_future();
    auto promise_notification_handle_2 = base_2->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise_2),
        Kinova::Api::Common::NotificationOptions());

    // std::cout << "Reaching joint angles..." << std::endl;
    base_1->PlayJointTrajectory(constrained_joint_angles);
    base_2->PlayJointTrajectory(constrained_joint_angles);

    // Wait for future value from promise (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status_1 = finish_future_1.wait_for(TIMEOUT_DURATION);
    const auto status_2 = finish_future_2.wait_for(TIMEOUT_DURATION);
    base_1->Unsubscribe(promise_notification_handle_1);
    base_2->Unsubscribe(promise_notification_handle_2);

    if (status_1 != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        std::cout << "Can't reach safe position, exiting" << std::endl;

        // Close API session
        session_manager_1->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_1->SetActivationStatus(false);
        transport_1->disconnect();

        // Destroy the API
        delete base_1;
        delete session_manager_1;
        delete router_1;
        delete transport_1;
        return -1;
    }

    if (status_2 != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        std::cout << "Can't reach safe position, exiting" << std::endl;

        // Close API session
        session_manager_2->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_2->SetActivationStatus(false);
        transport_2->disconnect();

        // Destroy the API
        delete base_2;
        delete session_manager_2;
        delete router_2;
        delete transport_2;
        return -1;
    }

    // const auto promise_event_1 = finish_future_1.get();
    // const auto promise_event_2 = finish_future_2.get();

    // std::cout << "Joint angles reached" << std::endl;
    // std::cout << "Promise value : " << Kinova::Api::Base::ActionEvent_Name(promise_event) << std::endl;

    // Close API session
    session_manager_1->CloseSession();
    session_manager_2->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router_1->SetActivationStatus(false);
    router_2->SetActivationStatus(false);

    transport_1->disconnect();
    transport_2->disconnect();

    // Destroy the API
    delete base_1;
    delete base_2;
    delete session_manager_1;
    delete session_manager_2;
    delete router_1;
    delete router_2;
    delete transport_1;
    delete transport_2;

    // printf("High-Level Control Completed\n");

    return 0;
}

int go_to_cart(kinova_manager &robot_driver_1, double speed_linear_1, double speed_angular_1, kinova_manager &robot_driver_2, double speed_linear_2, double speed_angular_2, const int desired_pose_)
{
    Pose desired_ee_pose_1(0, 0, 0, 0, 0, 0);
    Pose desired_ee_pose_2(0, 0, 0, 0, 0, 0);

    switch (desired_pose_) // Angle values are in units of degree
    {
    case desired_pose::CANDLE:
        desired_ee_pose_1 = Pose(0.057, -0.01, 1.0003, 0.0, 0.0, 90.0);
        desired_ee_pose_2 = Pose(0.057, -0.01, 1.0003, 0.0, 0.0, 90.0);
        break;
    case desired_pose::HOME:
        desired_ee_pose_1 = Pose(0.438, -0.195, 0.449, 90.0, 0.0, 30.0);
        desired_ee_pose_2 = Pose(0.438, -0.195, 0.449, 90.0, 0.0, 30.0);
        break;

    case desired_pose::TEST:
        desired_ee_pose_1 = Pose(0.0, -0.5, 0.427, 85.0, 0.0, 54.0);
        desired_ee_pose_2 = Pose(0.0, -0.5, 0.427, 85.0, 0.0, 54.0);
        break;

    case desired_pose::BLANKET_1:
        desired_ee_pose_1 = Pose(0.0, -0.5, 0.427, 85.0, 0.0, 54.0);
        desired_ee_pose_2 = Pose(0.2, 0.75, 0.311, 117.0, -98.0, 87.0);
        break;

    default:
        return -1;
    }

    auto error_callback = [](Kinova::Api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    //////////////////////////////////////////////////////////////////////

    // Kinova 1 Create API objects
    auto transport_1 = new Kinova::Api::TransportClientTcp();
    auto router_1 = new Kinova::Api::RouterClient(transport_1, error_callback);
    transport_1->connect(IP_ADDRESS_1, PORT);
    // Set session data connection information
    auto create_session_info_1 = Kinova::Api::Session::CreateSessionInfo();
    create_session_info_1.set_username("admin");
    create_session_info_1.set_password("admin");
    create_session_info_1.set_session_inactivity_timeout(200);    // (milliseconds)
    create_session_info_1.set_connection_inactivity_timeout(200); // (milliseconds)
    // Session manager service wrapper
    auto session_manager_1 = new Kinova::Api::SessionManager(router_1);
    session_manager_1->CreateSession(create_session_info_1);
    // Create services
    auto base_1 = new Kinova::Api::Base::BaseClient(router_1);
    auto base_cyclic_1 = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_1);

    //////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////

    // Kinova 2 Create API objects
    auto transport_2 = new Kinova::Api::TransportClientTcp();
    auto router_2 = new Kinova::Api::RouterClient(transport_2, error_callback);
    transport_2->connect(IP_ADDRESS_2, PORT);

    // Set session data connection information
    auto create_session_info_2 = Kinova::Api::Session::CreateSessionInfo();
    create_session_info_2.set_username("admin");
    create_session_info_2.set_password("admin");
    create_session_info_2.set_session_inactivity_timeout(200);    // (milliseconds)
    create_session_info_2.set_connection_inactivity_timeout(200); // (milliseconds)
    // Session manager service wrapper
    auto session_manager_2 = new Kinova::Api::SessionManager(router_2);
    session_manager_2->CreateSession(create_session_info_2);
    // Create services
    auto base_2 = new Kinova::Api::Base::BaseClient(router_2);
    auto base_cyclic_2 = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_2);

    //////////////////////////////////////////////////////////////////////

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_1->SetServoingMode(servoingMode);
    base_2->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // auto constrained_joint_angles = Kinova::Api::Base::ConstrainedJointAngles();
    // auto joint_angles = constrained_joint_angles.mutable_joint_angles();
    // auto actuator_count = base_1->GetActuatorCount();

    std::cout << "Starting Cartesian action movement ..." << std::endl;

    auto feedback_1 = base_cyclic_1->RefreshFeedback();
    auto action_1 = Kinova::Api::Base::Action();
    action_1.set_name("Kinova 1 Cartesian action movement");
    action_1.set_application_data("");

    // auto constrained_pose_1 = action_1.mutable_reach_pose();
    // auto pose_1 = constrained_pose_1->mutable_target_pose();
    // pose_1->set_x(desired_ee_pose_1[0]);       // x (meters)
    // pose_1->set_y(desired_ee_pose_1[1]);       // y (meters)
    // pose_1->set_z(desired_ee_pose_1[2]);       // z (meters)
    // pose_1->set_theta_x(desired_ee_pose_1[3]); // theta x (degrees)
    // pose_1->set_theta_y(desired_ee_pose_1[4]); // theta y (degrees)
    // pose_1->set_theta_z(desired_ee_pose_1[5]); // theta z (degrees)

    auto constrained_pose_1 = action_1.mutable_reach_pose();
    auto pose_1 = constrained_pose_1->mutable_target_pose();
    pose_1->set_x(desired_ee_pose_1.x);             // x (meters)
    pose_1->set_y(desired_ee_pose_1.y);             // y (meters)
    pose_1->set_z(desired_ee_pose_1.z);             // z (meters)
    pose_1->set_theta_x(desired_ee_pose_1.theta_x); // theta x (degrees)
    pose_1->set_theta_y(desired_ee_pose_1.theta_y); // theta y (degrees)
    pose_1->set_theta_z(desired_ee_pose_1.theta_z); // theta z (degrees)

    Kinova::Api::Base::CartesianSpeed *cartesian_speed_1 = constrained_pose_1->mutable_constraint()->mutable_speed();
    cartesian_speed_1->set_translation(speed_linear_1);  // Linear speed in m/s
    cartesian_speed_1->set_orientation(speed_angular_1); // Angular speed in deg/s

    //////////////////////////////////////////////////////////////////////

    auto feedback_2 = base_cyclic_2->RefreshFeedback();
    auto action_2 = Kinova::Api::Base::Action();
    action_2.set_name("Kinova 2 Cartesian action movement");
    action_2.set_application_data("");

    // auto constrained_pose_2 = action_2.mutable_reach_pose();
    // auto pose_2 = constrained_pose_2->mutable_target_pose();
    // pose_2->set_x(desired_ee_pose_2[0]);                  // x (meters)
    // pose_2->set_y((desired_ee_pose_2[1] + 0.06) * -1);    // y (meters)
    // pose_2->set_z(desired_ee_pose_2[2] + 0.05);           // z (meters)
    // pose_2->set_theta_x(desired_ee_pose_2[3]);            // theta x (degrees)
    // pose_2->set_theta_y(desired_ee_pose_2[4]);            // theta y (degrees)
    // pose_2->set_theta_z(abs(desired_ee_pose_2[5] - 180)); // theta z (degrees)

    // auto constrained_pose_2 = action_2.mutable_reach_pose();
    // auto pose_2 = constrained_pose_2->mutable_target_pose();
    // pose_2->set_x(desired_ee_pose_2[0]);            // x (meters)
    // pose_2->set_y(desired_ee_pose_2[1]);            // y (meters)
    // pose_2->set_z(desired_ee_pose_2[2]);            // z (meters)
    // pose_2->set_theta_x(desired_ee_pose_2[3]);      // theta x (degrees)
    // pose_2->set_theta_y(desired_ee_pose_2[4]);      // theta y (degrees)
    // pose_2->set_theta_z(abs(desired_ee_pose_2[5])); // theta z (degrees)

    auto constrained_pose_2 = action_2.mutable_reach_pose();
    auto pose_2 = constrained_pose_2->mutable_target_pose();
    pose_2->set_x(desired_ee_pose_2.x);             // x (meters)
    pose_2->set_y(desired_ee_pose_2.y);             // y (meters)
    pose_2->set_z(desired_ee_pose_2.z);             // z (meters)
    pose_2->set_theta_x(desired_ee_pose_2.theta_x); // theta x (degrees)
    pose_2->set_theta_y(desired_ee_pose_2.theta_y); // theta y (degrees)
    pose_2->set_theta_z(desired_ee_pose_2.theta_z); // theta z (degrees)

    Kinova::Api::Base::CartesianSpeed *cartesian_speed_2 = constrained_pose_2->mutable_constraint()->mutable_speed();
    cartesian_speed_2->set_translation(speed_linear_2);  // Linear speed in m/s
    cartesian_speed_2->set_orientation(speed_angular_2); // Angular speed in deg/s

    //////////////////////////////////////////////////////////////////////

    // Connect to notification action topic (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<Kinova::Api::Base::ActionEvent> finish_promise_1;
    auto finish_future_1 = finish_promise_1.get_future();
    auto promise_notification_handle_1 = base_1->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise_1),
        Kinova::Api::Common::NotificationOptions());
    std::promise<Kinova::Api::Base::ActionEvent> finish_promise_2;
    auto finish_future_2 = finish_promise_2.get_future();
    auto promise_notification_handle_2 = base_2->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise_2),
        Kinova::Api::Common::NotificationOptions());

    // // std::cout << "Reaching joint angles..." << std::endl;
    // base_1->PlayJointTrajectory(constrained_joint_angles);
    // base_2->PlayJointTrajectory(constrained_joint_angles);

    base_1->ExecuteAction(action_1);
    base_2->ExecuteAction(action_2);

    // Wait for future value from promise (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status_1 = finish_future_1.wait_for(TIMEOUT_DURATION);
    const auto status_2 = finish_future_2.wait_for(TIMEOUT_DURATION);
    base_1->Unsubscribe(promise_notification_handle_1);
    base_2->Unsubscribe(promise_notification_handle_2);

    if (status_1 != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        std::cout << "Can't reach safe position, exiting" << std::endl;

        // Close API session
        session_manager_1->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_1->SetActivationStatus(false);
        transport_1->disconnect();

        // Destroy the API
        delete base_1;
        delete base_cyclic_1;
        delete session_manager_1;
        delete router_1;
        delete transport_1;
        return -1;
    }

    if (status_2 != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        std::cout << "Can't reach safe position, exiting" << std::endl;

        // Close API session
        session_manager_2->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_2->SetActivationStatus(false);
        transport_2->disconnect();

        // Destroy the API
        delete base_2;
        delete base_cyclic_2;
        delete session_manager_2;
        delete router_2;
        delete transport_2;
        return -1;
    }

    // const auto promise_event_1 = finish_future_1.get();
    // const auto promise_event_2 = finish_future_2.get();

    // std::cout << "Joint angles reached" << std::endl;
    // std::cout << "Promise value : " << Kinova::Api::Base::ActionEvent_Name(promise_event) << std::endl;

    // Close API session
    session_manager_1->CloseSession();
    session_manager_2->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router_1->SetActivationStatus(false);
    router_2->SetActivationStatus(false);

    transport_1->disconnect();
    transport_2->disconnect();

    // Destroy the API
    delete base_1;
    delete base_2;
    delete session_manager_1;
    delete session_manager_2;
    delete router_1;
    delete router_2;
    delete transport_1;
    delete transport_2;

    // printf("High-Level Control Completed\n");

    return 0;
}

int gripper(kinova_manager &robot_driver_1, float position_1, kinova_manager &robot_driver_2, float position_2)
{
    auto error_callback = [](Kinova::Api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    //////////////////////////////////////////////////////////////////////

    // Kinova 1 Create API objects
    auto transport_1 = new Kinova::Api::TransportClientTcp();
    auto router_1 = new Kinova::Api::RouterClient(transport_1, error_callback);
    transport_1->connect(IP_ADDRESS_1, PORT);
    // Set session data connection information
    auto create_session_info_1 = Kinova::Api::Session::CreateSessionInfo();
    create_session_info_1.set_username("admin");
    create_session_info_1.set_password("admin");
    create_session_info_1.set_session_inactivity_timeout(200);    // (milliseconds)
    create_session_info_1.set_connection_inactivity_timeout(200); // (milliseconds)
    // Session manager service wrapper
    auto session_manager_1 = new Kinova::Api::SessionManager(router_1);
    session_manager_1->CreateSession(create_session_info_1);
    // Create services
    auto base_1 = new Kinova::Api::Base::BaseClient(router_1);
    auto base_cyclic_1 = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_1);

    //////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////

    // Kinova 2 Create API objects
    auto transport_2 = new Kinova::Api::TransportClientTcp();
    auto router_2 = new Kinova::Api::RouterClient(transport_2, error_callback);
    transport_2->connect(IP_ADDRESS_2, PORT);

    // Set session data connection information
    auto create_session_info_2 = Kinova::Api::Session::CreateSessionInfo();
    create_session_info_2.set_username("admin");
    create_session_info_2.set_password("admin");
    create_session_info_2.set_session_inactivity_timeout(200);    // (milliseconds)
    create_session_info_2.set_connection_inactivity_timeout(200); // (milliseconds)
    // Session manager service wrapper
    auto session_manager_2 = new Kinova::Api::SessionManager(router_2);
    session_manager_2->CreateSession(create_session_info_2);
    // Create services
    auto base_2 = new Kinova::Api::Base::BaseClient(router_2);
    auto base_cyclic_2 = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_2);

    //////////////////////////////////////////////////////////////////////

    // Kinova::Api::Base::GripperCommand gripper_command_1;

    // gripper_command_1.set_mode(Kinova::Api::Base::GRIPPER_POSITION);

    // auto finger_1 = gripper_command_1.mutable_gripper()->add_finger();
    // finger_1->set_finger_identifier(1);
    // for (float position = 0.0; position < position_1; position += 0.05)
    // {
    //     std::cout << "Setting position to " << position << std::endl;
    //     finger_1->set_value(position);
    //     base_1->SendGripperCommand(gripper_command_1);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    // Kinova::Api::Base::GripperCommand gripper_command_2;

    // gripper_command_2.set_mode(Kinova::Api::Base::GRIPPER_POSITION);

    // auto finger_2 = gripper_command_2.mutable_gripper()->add_finger();
    // finger_2->set_finger_identifier(1);
    // for (float position = 0.0; position < position_2; position += 0.05)
    // {
    //     std::cout << "Setting position to " << position << std::endl;
    //     finger_2->set_value(position);
    //     base_2->SendGripperCommand(gripper_command_2);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    Kinova::Api::Base::GripperCommand gripper_command_1, gripper_command_2;
    gripper_command_1.set_mode(Kinova::Api::Base::GRIPPER_POSITION);
    gripper_command_2.set_mode(Kinova::Api::Base::GRIPPER_POSITION);

    auto finger_1 = gripper_command_1.mutable_gripper()->add_finger();
    auto finger_2 = gripper_command_2.mutable_gripper()->add_finger();

    finger_1->set_finger_identifier(1);
    finger_2->set_finger_identifier(1);

    float step = 0.01;
    float max_position = std::max(position_1, position_2);

    for (float position = 0.0; position < max_position; position += step)
    {
        if (position < position_1)
        {
            std::cout << "Gripper 1: Setting position to " << position << std::endl;
            finger_1->set_value(position);
            base_1->SendGripperCommand(gripper_command_1);
        }

        if (position < position_2)
        {
            std::cout << "Gripper 2: Setting position to " << position << std::endl;
            finger_2->set_value(position);
            base_2->SendGripperCommand(gripper_command_2);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Synchronize movements
    }

    //////////////////////////////////////////////////////////////////////

    // Close API session
    session_manager_1->CloseSession();
    session_manager_2->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router_1->SetActivationStatus(false);
    router_2->SetActivationStatus(false);

    transport_1->disconnect();
    transport_2->disconnect();

    // Destroy the API
    delete base_1;
    delete base_2;
    delete session_manager_1;
    delete session_manager_2;
    delete router_1;
    delete router_2;
    delete transport_1;
    delete transport_2;

    return 0;
}

// Function definitions that handle errors internally
void go_to_with_error_handling(kinova_manager &robot_driver_1, kinova_manager &robot_driver_2, int desired_pose_id)
{
    int return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    if (return_flag != 0)
    {
        // Handle the error internally (log, retry, etc.)
        std::cerr << "Error in go_to function" << std::endl;
        return;
    }
}

void go_to_cart_with_error_handling(kinova_manager &robot_driver_1, double speed_linear_1, double speed_angular_1,
                                    kinova_manager &robot_driver_2, double speed_linear_2, double speed_angular_2,
                                    int desired_pose)
{
    int return_flag = go_to_cart(robot_driver_1, speed_linear_1, speed_angular_1, robot_driver_2,
                                 speed_linear_2, speed_angular_2, desired_pose);
    if (return_flag != 0)
    {
        // Handle the error internally (log, retry, etc.)
        std::cerr << "Error in go_to_cart function" << std::endl;
        return;
    }
}

void gripper_with_error_handling(kinova_manager &robot_driver_1, float position_1, kinova_manager &robot_driver_2, float position_2)
{
    int return_flag = gripper(robot_driver_1, position_1, robot_driver_2, position_2);
    if (return_flag != 0)
    {
        // Handle the error internally (log, retry, etc.)
        std::cerr << "Error in gripper function" << std::endl;
        return;
    }
}

int main(int argc, char **argv)
{
    RATE_HZ = 700; // Hz

    desired_pose_id = desired_pose::HOME;
    desired_control_mode = control_mode::POSITION;

    kinova_manager robot_driver_1;
    kinova_manager robot_driver_2;

    int return_flag = 0;

    // return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    // if (return_flag != 0)
    //     return 0;

    // return_flag = go_to_cart(robot_driver_1, robot_driver_2, desired_pose::BLANKET_1);
    // if (return_flag != 0)
    //     return 0;

    // return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    // if (return_flag != 0)
    //     return 0;

    // Extract robot model and if not simulation, establish connection with motor drivers
    if (!robot_driver_1.is_initialized())
        robot_driver_1.initialize(robot_id::KINOVA_GEN3_lITE_1, 1.0 / static_cast<double>(RATE_HZ));
    if (!robot_driver_1.is_initialized())
    {
        printf("Robot 1 is not initialized\n");
        return 0;
    }

    if (!robot_driver_2.is_initialized())
        robot_driver_2.initialize(robot_id::KINOVA_GEN3_lITE_2, 1.0 / static_cast<double>(RATE_HZ));
    if (!robot_driver_2.is_initialized())
    {
        printf("Robot 2 is not initialized\n");
        return 0;
    }

    // return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    // if (return_flag != 0)
    //     return 0;

    // return_flag = go_to_cart(robot_driver_1, 0.5, 10, robot_driver_2, 0.1, 10, desired_pose::BLANKET_1);
    // if (return_flag != 0)
    //     return 0;

    // return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    // if (return_flag != 0)
    //     return 0;

    // go_to_with_error_handling(robot_driver_1, robot_driver_2, desired_pose_id);
    go_to_cart_with_error_handling(robot_driver_1, 0.5, 10, robot_driver_2, 0.5, 10, desired_pose::BLANKET_1);
    gripper_with_error_handling(robot_driver_1, 0.2, robot_driver_2, 0.9);
    // go_to_with_error_handling(robot_driver_1, robot_driver_2, desired_pose_id);

    robot_driver_1.deinitialize();
    robot_driver_2.deinitialize();

    std::cout << "Kinova Multi-Arm Project Finish!" << std::endl;
    return 0;
}
