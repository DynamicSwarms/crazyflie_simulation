#pragma once

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_simulation/simulation/simulation.hpp"

#include "std_msgs/msg/string.hpp"

class Console
{
public:
    Console(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<Simulation> simulation
    );

private:
    void console_message(const std::string);
    
private:
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;

    std::weak_ptr<Simulation> m_simulation;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> m_console_publisher;

};