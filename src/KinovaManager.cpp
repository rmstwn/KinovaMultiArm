#include "KinovaManager.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <future>

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

KinovaManager::KinovaManager(const std::string &ip_address, int port)
    : is_connected_(false), connection_established_(false), ip_address_(ip_address), port_(port), transport_(nullptr), router_(nullptr),
      session_manager_(nullptr), control_config_(nullptr), base_(nullptr), base_cyclic_(nullptr)
{
    // error_callback_ = [](Kinova::Api::KError err)
    // {
    //     std::cout << "Error: " << err.toString() << std::endl;
    // };
    setupConnection();
}

KinovaManager::~KinovaManager()
{
    // Close API sessions and connections
    if (is_connected_)
        stopConnection();
}

bool KinovaManager::is_connected()
{
    return is_connected_;
}

void KinovaManager::setupConnection()
{
    // Reset Flags
    is_connected_ = false;

    auto error_callback = [](Kinova::Api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    // Create transport and router with the error callback
    transport_ = new Kinova::Api::TransportClientTcp();
    router_ = new Kinova::Api::RouterClient(transport_, error_callback);
    transport_->connect(ip_address_, port_);

    std::cout << "Kinova connected" << std::endl;

    base_ = new Kinova::Api::Base::BaseClient(router_);
    base_cyclic_ = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_);

    // Set session data connection information
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(200);
    create_session_info.set_connection_inactivity_timeout(200);

    // Session manager service wrapper
    session_manager_ = new Kinova::Api::SessionManager(router_);
    session_manager_->CreateSession(create_session_info);

    control_config_ = new Kinova::Api::ControlConfig::ControlConfigClient(router_);
    // auto gravityVector = new Kinova::Api::ControlConfig::GravityVector();

    // gravityVector->set_x(0.0);
    // gravityVector->set_y(0.0);
    // gravityVector->set_z(9.81);

    // gravityVector->x();
    // gravityVector->y();
    // gravityVector->z();

    // control_config_->GetGravityVector();

    // std::cout << gravityVector << std::endl;
    
    std::cout
        << "Kinova sessions created" << std::endl;

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
        is_connected_ = true; // Set initialization flag for the user
}

void KinovaManager::executePoseMovement(const std::vector<double> &configuration_array)
{
    auto constrained_joint_angles = Kinova::Api::Base::ConstrainedJointAngles();
    auto joint_angles = constrained_joint_angles.mutable_joint_angles();

    for (size_t i = 0; i < configuration_array.size(); ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(configuration_array[i]);
    }

    auto finish_promise = std::promise<Kinova::Api::Base::ActionEvent>();
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base_->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise), Kinova::Api::Common::NotificationOptions());

    base_->PlayJointTrajectory(constrained_joint_angles);

    finish_future.wait_for(std::chrono::seconds(30)); // Adjust this duration as necessary

    base_->Unsubscribe(promise_notification_handle);
}

void KinovaManager::executeCartesianMovement(const std::vector<double> &desired_ee_pose, double &speed_linear, double &speed_angular)
{
    // Implement Cartesian movement execution logic here
    // You can use the desired_ee_pose for Cartesian control
    // Similar to joint angle control, but based on position/rotation instead

    std::cout << "Starting Cartesian action movement ..." << std::endl;

    auto feedback_ = base_cyclic_->RefreshFeedback();
    auto action_ = Kinova::Api::Base::Action();
    action_.set_name("Kinova 1 Cartesian action movement");
    action_.set_application_data("");

    auto constrained_pose_ = action_.mutable_reach_pose();
    auto pose_ = constrained_pose_->mutable_target_pose();

    pose_->set_x(desired_ee_pose[0]);       // x (meters)
    pose_->set_y(desired_ee_pose[1]);       // y (meters)
    pose_->set_z(desired_ee_pose[2]);       // z (meters)
    pose_->set_theta_x(desired_ee_pose[3]); // theta x (degrees)
    pose_->set_theta_y(desired_ee_pose[4]); // theta y (degrees)
    pose_->set_theta_z(desired_ee_pose[5]); // theta z (degrees)

    Kinova::Api::Base::CartesianSpeed *cartesian_speed_ = constrained_pose_->mutable_constraint()->mutable_speed();
    cartesian_speed_->set_translation(speed_linear);  // Linear speed in m/s
    cartesian_speed_->set_orientation(speed_angular); // Angular speed in deg/s

    auto finish_promise = std::promise<Kinova::Api::Base::ActionEvent>();
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base_->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise), Kinova::Api::Common::NotificationOptions());

    base_->ExecuteAction(action_);

    finish_future.wait_for(std::chrono::seconds(30)); // Adjust this duration as necessary

    base_->Unsubscribe(promise_notification_handle);
}

int KinovaManager::go_to(const int desired_pose)
{
    std::vector<double> configuration_array(6, 0.0);
    switch (desired_pose)
    {
    case desired_pose::CANDLE: // CANDLE
        configuration_array = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        break;
    case desired_pose::APPROACH_TABLE: // APPROACH_TABLE
        configuration_array = {0.001, 42.017, 179.56, 220.641, 2.761, 1.965};
        break;
    case desired_pose::HOME_BACK: // HOME_BACK
        configuration_array = {356.129, 304.126, 181.482, 250.087, 2.852, 328.367};
        break;
    case desired_pose::PACKAGING: // PACKAGING
        configuration_array = {270.0, 148.0, 148.0, 270.0, 140.0, 0.0};
        break;
    case desired_pose::RETRACT: // RETRACT
        configuration_array = {0.0, 340.0, 180.0, 214.0, 0.0, 310.0};
        break;
    case desired_pose::HOME: // HOME
        configuration_array = {0.0, 344.0, 75.0, 0.0, 300.0, 0.0};
        break;
    default:
        return -1;
    }

    executePoseMovement(configuration_array);
    return 0;
}

int KinovaManager::go_to_cart(double speed_linear, double speed_angular, const int desired_pose)
{
    std::vector<double> desired_ee_pose(6, 0.0);

    switch (desired_pose)
    {
    case desired_pose::CANDLE: // CANDLE
        desired_ee_pose = {0.057, -0.01, 1.0003, 0.0, 0.0, 90.0};
        break;
    case desired_pose::HOME: // HOME
        desired_ee_pose = {0.438, -0.195, 0.449, 90.0, 0.0, 30.0};
        break;
    default:
        return -1;
    }

    executeCartesianMovement(desired_ee_pose, speed_linear, speed_angular);
    return 0;
}

void KinovaManager::stopConnection()
{
    // Necessary to avoid hard restart of the arm for the next control trial
    stop_robot_motion();

    // Check if the session manager exists before trying to stop the connection
    if (session_manager_)
    {
        try
        {
            // Clear the session
            session_manager_->CloseSession();
            std::cout << "Session closed successfully." << std::endl;
        }
        catch (const std::exception &ex)
        {
            std::cout << "Failed to close session: " << ex.what() << std::endl;
        }
    }

    // Disconnect the transport
    if (transport_)
    {
        try
        {
            transport_->disconnect();
            std::cout << "Transport disconnected successfully." << std::endl;
        }
        catch (const std::exception &ex)
        {
            std::cout << "Failed to disconnect transport: " << ex.what() << std::endl;
        }
    }

    // Delete API objects to clean up resources
    delete session_manager_;
    delete base_;
    delete base_cyclic_;
    delete router_;
    delete transport_;
    delete control_config_;

    session_manager_ = nullptr;
    base_ = nullptr;
    router_ = nullptr;
    transport_ = nullptr;

    is_connected_ = false;
    std::cout << "API components have been deleted and cleaned up." << std::endl;
}

int KinovaManager::stop_robot_motion()
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


int KinovaManager::gripper(float target_position, int64_t time)
{
    // Initialize gripper command
    Kinova::Api::Base::GripperCommand gripper_command_;
    gripper_command_.set_mode(Kinova::Api::Base::GRIPPER_POSITION);
    
    // Set the initial position of the gripper to a known state (finger 1)
    auto finger_ = gripper_command_.mutable_gripper()->add_finger();
    finger_->set_finger_identifier(1);
    finger_->set_value(0); // Set initial position to 0 (open)
    
    // Send the initial gripper position
    base_->SendGripperCommand(gripper_command_);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));  // Wait for initialization

    // Move the gripper to the target position
    gripper_command_.set_mode(Kinova::Api::Base::GRIPPER_POSITION);
    finger_->set_value(target_position);
    base_->SendGripperCommand(gripper_command_);
    std::cout << "⏳ Moving gripper to target position: " << target_position << std::endl;

    // Feedback loop to check gripper position
    Kinova::Api::Base::Gripper gripper_feedback;
    Kinova::Api::Base::GripperRequest gripper_request;
    bool is_motion_completed = false;

    // Set the request mode to position for feedback
    gripper_request.set_mode(Kinova::Api::Base::GRIPPER_POSITION);

    // Continue to check feedback until the gripper reaches the target position or time-out
    while (!is_motion_completed)
    {
        float position_ = 0.0f;

        // Get the feedback on gripper position
        gripper_feedback = base_->GetMeasuredGripperMovement(gripper_request);

        // Ensure the finger position is available in the feedback
        if (gripper_feedback.finger_size() > 0)
        {
            position_ = gripper_feedback.finger(0).value();
            std::cout << "Reported position: " << position_ << std::endl;
        }
        else
        {
            std::cerr << "❌ Gripper feedback not available!" << std::endl;
            return -1;
        }

        // Check if the gripper position is within tolerance of the target
        if (std::abs(position_ - target_position) <= 0.05)  // Tolerance threshold of 0.05
        {
            is_motion_completed = true;
            std::cout << "✅ Gripper has reached the target position: " << target_position << std::endl;
        }
        else
        {
            std::cout << "🔄 Gripper is still moving. Current position: " << position_ << std::endl;
        }

        // Wait before re-checking
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;  // Success
}
