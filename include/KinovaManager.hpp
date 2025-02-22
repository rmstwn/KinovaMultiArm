#ifndef KINOVA_MANAGER_HPP
#define KINOVA_MANAGER_HPP

#include <vector>
#include <future>
#include <chrono>

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
#include <ControlConfigClientRpc.h>


#include <google/protobuf/util/json_util.h>

class KinovaManager
{
public:
    KinovaManager(const std::string &ip_address, int port);
    ~KinovaManager();

    virtual bool is_connected();

    int go_to(const int desired_pose);
    // int go_to_cart(double speed_linear, double speed_angular, const int desired_pose);
    int go_to_cart(double speed_linear, double speed_angular, const std::vector<double> &desired_pose);


    int gripper(float target_position, int64_t time);

    void setupConnection();
    void stopConnection();

    virtual int stop_robot_motion();

private:
    // void executePoseMovement(const std::vector<double> &configuration_array);
    // void executeCartesianMovement(const std::vector<double> &desired_ee_pose, const std::vector<double> &speed);
    // void executeCartesianMovement(const std::vector<double> &desired_ee_pose, double &speed_linear, double &speed_angular);

    void executePoseMovement(const std::vector<double> &configuration_array);
    void executeCartesianMovement(const std::vector<double> &desired_ee_pose, double &speed_linear, double &speed_angular);


    
    bool is_connected_;
    bool connection_established_;

    std::string ip_address_;
    int port_;
    Kinova::Api::TransportClientTcp *transport_;
    Kinova::Api::RouterClient *router_;
    Kinova::Api::SessionManager *session_manager_;
    Kinova::Api::Base::BaseClient *base_;
    Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic_;
    Kinova::Api::ControlConfig::ControlConfigClient *control_config_;

    Kinova::Api::Base::ServoingModeInformation servoing_mode_;

    std::function<void(Kinova::Api::KError)> error_callback_;
};

#endif // KINOVA_MANAGER_HPP
