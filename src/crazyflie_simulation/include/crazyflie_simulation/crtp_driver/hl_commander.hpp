

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_simulation/simulation/simulation.hpp"

#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"


class HighLevelCommander {
public:
    HighLevelCommander(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<Simulation> simulation
    );

private: 

    void land_service(
        const crazyflie_interfaces::srv::Land::Request::SharedPtr request, 
        crazyflie_interfaces::srv::Land::Response::SharedPtr response);

    void takeoff_service(
        const crazyflie_interfaces::srv::Takeoff::Request::SharedPtr request, 
        crazyflie_interfaces::srv::Takeoff::Response::SharedPtr response);

    void goto_service(
        const crazyflie_interfaces::srv::GoTo::Request::SharedPtr request, 
        crazyflie_interfaces::srv::GoTo::Response::SharedPtr response);

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;

    std::weak_ptr<Simulation> m_simulation;
    
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::Land>> m_land_service;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::Takeoff>> m_takeoff_service;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::GoTo>> m_goto_service;
};