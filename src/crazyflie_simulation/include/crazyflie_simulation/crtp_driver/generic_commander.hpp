

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_simulation/simulation/simulation.hpp"

#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"


class GenericCommander  {
public:
    GenericCommander(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<Simulation> simulation
    );

private: 

    void cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg);
    
    void notify_setpoints_stop_service(
        const crazyflie_interfaces::srv::NotifySetpointsStop::Request::SharedPtr request, 
        crazyflie_interfaces::srv::NotifySetpointsStop::Response::SharedPtr response);
    

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;
    std::weak_ptr<Simulation> m_simulation;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group; 
    std::shared_ptr<rclcpp::Subscription<crazyflie_interfaces::msg::Position>> m_cmd_position_sub;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::NotifySetpointsStop>> m_notify_setpoints_stop_service;
};