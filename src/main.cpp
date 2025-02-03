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
    APPROACH_TABLE = 6
};

enum path_types
{
    SINE_PATH = 0,
    STEP_PATH = 1,
    INF_SIGN_PATH = 2
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
std::function<void(Kinova::Api::Base::ActionNotification)> create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (Kinova::Api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
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
    std::cout << serialized_data << std::endl << std::endl;
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

int main(int argc, char **argv)
{
    RATE_HZ = 700; // Hz

    kinova_manager robot_driver_1;
    kinova_manager robot_driver_2;

    int return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    if (return_flag != 0)
        return 0;

    std::cout << "Kinova Multi-Arm Project Initialized!" << std::endl;
    return 0;
}
