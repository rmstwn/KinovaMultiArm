/*
Author(s): Muhammad Ramadhan Hadi Setyawan
Institute: Takesue Laboratory, Tokyo Metropolitan University
Description:

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

#include "kinova_manager.hpp"

#define IP_ADDRESS_1 "192.168.2.10"
#define IP_ADDRESS_2 "192.168.2.11"
#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 6
#define SEGMENT_COUNT_FULL 8
#define NUM_OF_CONSTRAINTS 5

kinova_manager::kinova_manager() : is_initialized_(false), kinova_id(robot_id::KINOVA_GEN3_lITE_1),
                                   control_mode_(control_mode::STOP_MOTION),
                                   add_offsets_(false), connection_established_(false), DT_SEC_(0.0),
                                   ext_wrenches_sim_(SEGMENT_COUNT_FULL, KDL::Wrench::Zero()),
                                   robot_state_(ACTUATOR_COUNT, SEGMENT_COUNT_FULL, SEGMENT_COUNT_FULL + 1, NUM_OF_CONSTRAINTS),
                                   predicted_states_(1, robot_state_),
                                   transport_(nullptr), transport_real_time_(nullptr), router_(nullptr),
                                   router_real_time_(nullptr), session_manager_(nullptr),
                                   session_manager_real_time_(nullptr), base_(nullptr),
                                   base_cyclic_(nullptr), actuator_config_(nullptr)
{
    // Removed initialization for joint_inertia_sim_
}

kinova_manager::~kinova_manager()
{
    // Close API sessions and connections
    if (is_initialized_)
        deinitialize();
}

// Update robot state: measured positions, velocities, torques and measured / estimated external forces on end-effector
void kinova_manager::get_robot_state(KDL::JntArray &joint_positions,
                                     KDL::JntArray &joint_velocities,
                                     KDL::JntArray &joint_torques,
                                     KDL::Wrench &end_effector_wrench)
{
    get_joint_state(joint_positions, joint_velocities, joint_torques);
    // get_end_effector_wrench(end_effector_wrench);
}

// Update joint space state: measured positions, velocities and torques
void kinova_manager::get_joint_state(KDL::JntArray &joint_positions,
                                     KDL::JntArray &joint_velocities,
                                     KDL::JntArray &joint_torques)
{

    try
    {
        base_feedback_ = base_cyclic_->RefreshFeedback();
    }
    catch (Kinova::Api::KDetailedException &ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
    }

    get_joint_positions(joint_positions);
    get_joint_velocities(joint_velocities);
    // get_joint_torques(joint_torques);
}

// Get Joint Positions
void kinova_manager::get_joint_positions(KDL::JntArray &joint_positions)
{
    // Joint position given in deg
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        joint_positions(i) = DEG_TO_RAD(base_feedback_.actuators(i).position());

    // Kinova API provides only positive angle values
    // This operation is required to align the logic with our safety monitor
    // We need to convert some angles to negative values
    if (joint_positions(1) > DEG_TO_RAD(180.0))
        joint_positions(1) -= DEG_TO_RAD(360.0);
    if (joint_positions(3) > DEG_TO_RAD(180.0))
        joint_positions(3) -= DEG_TO_RAD(360.0);
    if (joint_positions(5) > DEG_TO_RAD(180.0))
        joint_positions(5) -= DEG_TO_RAD(360.0);
}

// Set Joint Positions
int kinova_manager::set_joint_positions(const KDL::JntArray &joint_positions)
{
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        base_command_.mutable_actuators(i)->set_position(RAD_TO_DEG(joint_positions(i)));
    }

    increment_command_id();

    // Send the commands
    try
    {
        base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
    }
    catch (Kinova::Api::KDetailedException &ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        return -1;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "runtime error: " << ex2.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cout << "Unknown error." << std::endl;
        return -1;
    }

    return 0;
}

// Get Joint Velocities
void kinova_manager::get_joint_velocities(KDL::JntArray &joint_velocities)
{
    // Joint velocity given in deg/sec
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        joint_velocities(i) = DEG_TO_RAD(base_feedback_.actuators(i).velocity());
}

// Set Joint Velocities
int kinova_manager::set_joint_velocities(const KDL::JntArray &joint_velocities)
{
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        base_command_.mutable_actuators(i)->set_velocity(RAD_TO_DEG(joint_velocities(i)));
    }

    increment_command_id();

    // Send the commands
    try
    {
        base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
    }
    catch (Kinova::Api::KDetailedException &ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        return -1;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "runtime error: " << ex2.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cout << "Unknown error." << std::endl;
        return -1;
    }

    return 0;
}

// Set Kinova control mode
// int kinova_manager::set_control_mode(const int desired_control_mode)
// {
//     control_mode_ = desired_control_mode;

//     try
//     {
//         switch (control_mode_)
//         {
//         case control_mode::TORQUE:
//             // Set actuators in torque mode
//             control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
//             for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
//                 actuator_config_->SetControlMode(control_mode_message_, actuator_id);
//             return 0;

//         case control_mode::VELOCITY:
//             // Set actuators in velocity mode
//             control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::VELOCITY);
//             for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
//                 actuator_config_->SetControlMode(control_mode_message_, actuator_id);
//             return 0;

//         case control_mode::POSITION:
//             // Set actuators in position mode
//             control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
//             for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
//                 actuator_config_->SetControlMode(control_mode_message_, actuator_id);
//             return 0;

//         default:
//             assert(("Unknown control mode!", false));
//             return -1;
//         }
//     }
//     catch (Kinova::Api::KDetailedException &ex)
//     {
//         std::cout << "Error here start" << std::endl;
//         std::cout << "Kortex exception: " << ex.what() << std::endl;
//         std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
//         std::cout << "Error here end" << std::endl;
//         return -1;
//     }
//     catch (std::runtime_error &ex2)
//     {
//         std::cout << "runtime error: " << ex2.what() << std::endl;
//         return -1;
//     }
//     catch (...)
//     {
//         std::cout << "Unknown error." << std::endl;
//         return -1;
//     }

//     return 0;
// }

int kinova_manager::set_control_mode(const int desired_control_mode)
{
    try
    {
        // Check the current servoing mode
        Kinova::Api::Base::ServoingModeInformation servoing_mode_info = base_->GetServoingMode();

        // If not in LOW_LEVEL_SERVOING, switch it
        if (servoing_mode_info.servoing_mode() != Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING)
        {
            std::cout << "[INFO] Robot is not in LOW_LEVEL_SERVOING mode. Switching now..." << std::endl;

            auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();
            servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(servoing_mode);

            // Wait longer to ensure the change takes effect
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // Verify mode change
            servoing_mode_info = base_->GetServoingMode();
            if (servoing_mode_info.servoing_mode() != Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING)
            {
                std::cerr << "[ERROR] Failed to switch to LOW_LEVEL_SERVOING mode!" << std::endl;
                return -1;
            }
            std::cout << "[INFO] Successfully switched to LOW_LEVEL_SERVOING mode." << std::endl;
        }

        // Now proceed to set control mode
        control_mode_ = desired_control_mode;
        std::cout << "[INFO] Setting control mode to: " << control_mode_ << std::endl;

        // Assign appropriate control mode
        switch (control_mode_)
        {
        case control_mode::TORQUE:
            control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
            break;
        case control_mode::VELOCITY:
            control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::VELOCITY);
            break;
        case control_mode::POSITION:
            control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
            break;
        default:
            std::cerr << "[ERROR] Unknown control mode!" << std::endl;
            return -1;
        }

        // Apply the control mode to all actuators
        for (int actuator_id = 1; actuator_id <= ACTUATOR_COUNT; actuator_id++)
        {
            actuator_config_->SetControlMode(control_mode_message_, actuator_id);
            std::cout << "[INFO] Set control mode for actuator " << actuator_id << std::endl;
        }

        return 0;
    }
    catch (Kinova::Api::KDetailedException &ex)
    {
        std::cout << "Error here start" << std::endl;
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error code: " << ex.getErrorInfo().getError().error_code() << std::endl;
        std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code()))
                  << std::endl;
        std::cout << "Error here end" << std::endl;
        return -1;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "[ERROR] runtime error: " << ex2.what() << std::endl;
        return -1;
    }
    catch (...)
    {
        std::cout << "[ERROR] Unknown error." << std::endl;
        return -1;
    }
}

int kinova_manager::set_joint_command(const KDL::JntArray &joint_positions,
                                      const KDL::JntArray &joint_velocities,
                                      const KDL::JntArray &joint_torques,
                                      const int desired_control_mode)
{
    assert(joint_positions.rows() == kinova_constants::NUMBER_OF_JOINTS);
    assert(joint_velocities.rows() == kinova_constants::NUMBER_OF_JOINTS);
    assert(joint_torques.rows() == kinova_constants::NUMBER_OF_JOINTS);

    switch (desired_control_mode)
    {
        // case control_mode::TORQUE:
        //     if (control_mode_ != control_mode::TORQUE)
        //         set_control_mode(desired_control_mode);
        //     return set_joint_torques(joint_torques);

    case control_mode::VELOCITY:
        if (control_mode_ != control_mode::VELOCITY)
            set_control_mode(desired_control_mode);
        return set_joint_velocities(joint_velocities);

    case control_mode::POSITION:
        if (control_mode_ != control_mode::POSITION)
            set_control_mode(desired_control_mode);
        return set_joint_positions(joint_positions);

    default:
        assert(("Unknown control mode!", false));
        return -1;
    }

    return 0;
}

bool kinova_manager::robot_stopped()
{
    // Check if velocity control mode is active
    // if (control_mode_message_.control_mode() != Kinova::Api::ActuatorConfig::ControlMode::VELOCITY) return false;

    base_feedback_ = base_cyclic_->RefreshFeedback();
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        // Check if velocity setpoint is zero
        if ((base_feedback_.actuators(i).velocity() != 0.0) ||
            !std::isfinite(base_feedback_.actuators(i).velocity()))
            return false;
    }
    return true;
}

// Set Zero Joint Velocities and wait until robot has stopped completely
// int kinova_manager::stop_robot_motion()
// {

//     base_feedback_ = base_cyclic_->RefreshFeedback();

//     for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
//         base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());

//     if (control_mode_ != control_mode::POSITION)
//         set_control_mode(control_mode::POSITION);

//     increment_command_id();

//     // Send the commands
//     try
//     {
//         base_feedback_ = base_cyclic_->Refresh(base_command_, 0);
//     }
//     catch (Kinova::Api::KDetailedException &ex)
//     {
//         std::cout << "Kortex exception: " << ex.what() << std::endl;
//         std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
//         return -1;
//     }
//     catch (std::runtime_error &ex2)
//     {
//         std::cout << "runtime error: " << ex2.what() << std::endl;
//         return -1;
//     }
//     catch (...)
//     {
//         std::cout << "Unknown error." << std::endl;
//         return -1;
//     }

//     return 0;
// }

int kinova_manager::stop_robot_motion()
{
    try
    {
        base_->Stop();
        std::cout << "Robot motion stopped successfully in SINGLE_LEVEL_SERVOING mode." << std::endl;
    }
    catch (Kinova::Api::KDetailedException &ex)
    {
        std::cout << "Kortex exception: " << ex.what() << std::endl;
        std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "runtime error: " << ex2.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "Unknown error while stopping the robot." << std::endl;
    }

    return 0;
}

// Increses index of the command's frame id (buffer)
void kinova_manager::increment_command_id()
{
    // Incrementing identifier ensures actuators can reject out of time frames
    // Buffer?
    base_command_.set_frame_id(base_command_.frame_id() + 1);
    if (base_command_.frame_id() > 65535)
        base_command_.set_frame_id(0);

    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        base_command_.mutable_actuators(i)->set_command_id(base_command_.frame_id());
}

std::vector<double> kinova_manager::get_maximum_joint_pos_limits()
{
    return kinova_constants::joint_position_limits_max;
}

std::vector<double> kinova_manager::get_minimum_joint_pos_limits()
{
    return kinova_constants::joint_position_limits_min;
}

std::vector<double> kinova_manager::get_joint_position_thresholds()
{
    return kinova_constants::joint_position_thresholds;
}

std::vector<double> kinova_manager::get_joint_velocity_limits()
{
    return kinova_constants::joint_velocity_limits;
}

std::vector<double> kinova_manager::get_joint_acceleration_limits()
{
    assert(ACTUATOR_COUNT == kinova_constants::joint_acceleration_limits.size());
    return kinova_constants::joint_acceleration_limits;
}

std::vector<double> kinova_manager::get_joint_torque_limits()
{
    return kinova_constants::joint_torque_limits;
}

std::vector<double> kinova_manager::get_joint_stopping_torque_limits()
{
    assert(ACTUATOR_COUNT == kinova_constants::joint_stopping_torque_limits.size());
    return kinova_constants::joint_stopping_torque_limits;
}

std::vector<double> kinova_manager::get_joint_inertia()
{
    return kinova_constants::joint_inertia;
}

std::vector<double> kinova_manager::get_joint_offsets()
{
    return kinova_constants::joint_offsets;
}

KDL::Twist kinova_manager::get_root_acceleration()
{
    if (kinova_id == robot_id::KINOVA_GEN3_lITE_1)
        return KDL::Twist(KDL::Vector(kinova_constants::root_acceleration_1[0],
                                      kinova_constants::root_acceleration_1[1],
                                      kinova_constants::root_acceleration_1[2]),
                          KDL::Vector(kinova_constants::root_acceleration_1[3],
                                      kinova_constants::root_acceleration_1[4],
                                      kinova_constants::root_acceleration_1[5]));
    else
        return KDL::Twist(KDL::Vector(kinova_constants::root_acceleration_2[0],
                                      kinova_constants::root_acceleration_2[1],
                                      kinova_constants::root_acceleration_2[2]),
                          KDL::Vector(kinova_constants::root_acceleration_2[3],
                                      kinova_constants::root_acceleration_2[4],
                                      kinova_constants::root_acceleration_2[5]));
}

int kinova_manager::get_robot_ID()
{
    return kinova_id;
}

bool kinova_manager::is_initialized()
{
    return is_initialized_;
}

// Initialize variables and calibrate the manipulator:
// void kinova_manager::initialize(const int id,
//                                 const double DT_SEC)
// {
//     kinova_id = id;
//     DT_SEC_ = DT_SEC;
//     kinova_chain_ = KDL::Chain();

//     // Reset Flags
//     is_initialized_ = false;
//     add_offsets_ = false;

//     // If the real robot is controlled, settup the connection
//     // Create API error-callback and objects
//     // Connect all ports for real control
//     auto error_callback = [](Kinova::Api::KError err)
//     { cout << "_________ callback error _________" << err.toString(); };
//     this->transport_ = std::make_shared<Kinova::Api::TransportClientTcp>();
//     this->router_ = std::make_shared<Kinova::Api::RouterClient>(transport_.get(), error_callback);
//     if (kinova_id == KINOVA_GEN3_lITE_1)
//         transport_->connect(IP_ADDRESS_1, PORT);
//     else
//         transport_->connect(IP_ADDRESS_2, PORT);

//     this->transport_real_time_ = std::make_shared<Kinova::Api::TransportClientUdp>();
//     this->router_real_time_ = std::make_shared<Kinova::Api::RouterClient>(transport_real_time_.get(), error_callback);
//     if (kinova_id == KINOVA_GEN3_lITE_1)
//         transport_real_time_->connect(IP_ADDRESS_1, PORT_REAL_TIME);
//     else
//         transport_real_time_->connect(IP_ADDRESS_2, PORT_REAL_TIME);

//     // Set session data connection information
//     auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
//     create_session_info.set_username("admin");
//     create_session_info.set_password("admin");
//     create_session_info.set_session_inactivity_timeout(200);    // (milliseconds)
//     create_session_info.set_connection_inactivity_timeout(200); // (milliseconds)

//     // Session manager service wrapper
//     this->session_manager_ = std::make_shared<Kinova::Api::SessionManager>(router_.get());
//     session_manager_->CreateSession(create_session_info);

//     this->session_manager_real_time_ = std::make_shared<Kinova::Api::SessionManager>(router_real_time_.get());
//     session_manager_real_time_->CreateSession(create_session_info);

//     // Create services
//     this->base_ = std::make_shared<Kinova::Api::Base::BaseClient>(router_.get());
//     this->base_cyclic_ = std::make_shared<Kinova::Api::BaseCyclic::BaseCyclicClient>(router_real_time_.get());
//     this->actuator_config_ = std::make_shared<Kinova::Api::ActuatorConfig::ActuatorConfigClient>(router_.get());

//     std::cout << "Kinova sessions created" << std::endl;

//     // Clearing faults
//     try
//     {
//         base_->ClearFaults();
//     }
//     catch (...)
//     {
//         std::cout << "Unable to clear robot faults" << std::endl;
//         return;
//     }

//     // Initializing actuators
//     try
//     {
//         // Set the robot in low-level servoing mode
//         servoing_mode_.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
//         base_->SetServoingMode(servoing_mode_);

//         // Wait
//         std::this_thread::sleep_for(std::chrono::milliseconds(500));

//         // Get the initial state
//         base_feedback_ = base_cyclic_->RefreshFeedback();

//         std::cout << "Initialize each actuator to their current position" << std::endl;

//         // Initialize each actuator to their current position
//         for (int i = 0; i < ACTUATOR_COUNT; i++)
//             base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());

//         // Send a first command (time frame) -> position command in this case
//         base_feedback_ = base_cyclic_->Refresh(base_command_, 0);

//         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     }
//     catch (Kinova::Api::KDetailedException &ex)
//     {
//         std::cout << "API error: " << ex.what() << std::endl;

//         std::cout << "KError error_code: " << ex.getErrorInfo().getError().error_code() << std::endl;
//         std::cout << "KError sub_code: " << ex.getErrorInfo().getError().error_sub_code() << std::endl;
//         std::cout << "KError sub_string: " << ex.getErrorInfo().getError().error_sub_string() << std::endl;

//         // Error codes by themselves are not very verbose if you don't see their corresponding enum value
//         // You can use google::protobuf helpers to get the string enum element for every error code and sub-code
//         std::cout << "Error code string equivalent: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(ex.getErrorInfo().getError().error_code())) << std::endl;
//         std::cout << "Error sub-code string equivalent: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;

//         return;
//     }
//     catch (std::runtime_error &ex2)
//     {
//         std::cout << "Run-time Error: " << ex2.what() << std::endl;
//         return;
//     }
//     catch (...)
//     {
//         std::cout << "Unknown error" << std::endl;
//         return;
//     }

//     // Set connection flag
//     connection_established_ = true;

//     if (!connection_established_)
//         printf("Cannot create Kinova model! \n");
//     else
//         is_initialized_ = true; // Set initialization flag for the user
// }

void kinova_manager::initialize(const int id, const double DT_SEC)
{
    kinova_id = id;
    DT_SEC_ = DT_SEC;
    kinova_chain_ = KDL::Chain();

    // Reset Flags
    is_initialized_ = false;
    add_offsets_ = false;

    // If the real robot is controlled, setup the connection
    auto error_callback = [](Kinova::Api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    this->transport_ = std::make_shared<Kinova::Api::TransportClientTcp>();
    this->router_ = std::make_shared<Kinova::Api::RouterClient>(transport_.get(), error_callback);

    if (kinova_id == KINOVA_GEN3_lITE_1)
        transport_->connect(IP_ADDRESS_1, PORT);
    else
        transport_->connect(IP_ADDRESS_2, PORT);

    // Set session data connection information
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(200);
    create_session_info.set_connection_inactivity_timeout(200);

    // Session manager service wrapper
    this->session_manager_ = std::make_shared<Kinova::Api::SessionManager>(router_.get());
    session_manager_->CreateSession(create_session_info);

    this->base_ = std::make_shared<Kinova::Api::Base::BaseClient>(router_.get());

    std::cout << "Kinova sessions created" << std::endl;

    // Clearing faults
    try
    {
        base_->ClearFaults();
    }
    catch (...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return;
    }

    // Set the robot in single-level servoing mode for Cartesian control
    try
    {
        servoing_mode_.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_->SetServoingMode(servoing_mode_);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "Robot is now in SINGLE_LEVEL_SERVOING mode (Cartesian motion enabled)." << std::endl;
    }
    catch (Kinova::Api::KDetailedException &ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        std::cout << "KError error_code: " << ex.getErrorInfo().getError().error_code() << std::endl;
        std::cout << "Error sub-code string: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
        return;
    }
    catch (...)
    {
        std::cout << "Unknown error" << std::endl;
        return;
    }

    // Set connection flag
    connection_established_ = true;

    if (!connection_established_)
        std::cout << "Cannot create Kinova model!" << std::endl;
    else
        is_initialized_ = true; // Set initialization flag for the user
}

// void kinova_manager::deinitialize()
// {
//     // Necessary to avoid hard restart of the arm for the next control trial
//     stop_robot_motion();

//     // Close API session
//     session_manager_->CloseSession();
//     session_manager_real_time_->CloseSession();

//     // Deactivate the router and cleanly disconnect from the transport object
//     router_->SetActivationStatus(false);
//     transport_->disconnect();
//     router_real_time_->SetActivationStatus(false);
//     transport_real_time_->disconnect();

//     is_initialized_ = false;
//     printf("Robot deinitialized! \n\n\n");
// }

void kinova_manager::deinitialize()
{
    // Necessary to avoid hard restart of the arm for the next control trial
    stop_robot_motion();

    // Close API session safely
    if (session_manager_)
        session_manager_->CloseSession();
    if (session_manager_real_time_)
        session_manager_real_time_->CloseSession();

    // Deactivate transport safely
    if (transport_)
    {
        transport_->disconnect();
        if (router_)
            router_->SetActivationStatus(false);
    }

    if (transport_real_time_)
    {
        transport_real_time_->disconnect();
        if (router_real_time_)
            router_real_time_->SetActivationStatus(false);
    }

    is_initialized_ = false;
    printf("Robot deinitialized! \n\n\n");
}
