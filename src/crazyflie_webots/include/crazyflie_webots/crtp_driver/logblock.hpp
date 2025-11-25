#pragma once

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/msg/generic_log_data.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int16.hpp"

class LogBlock {

using GetDataCallback = std::function<std::vector<double>(const std::string &block_name)>;

public: 
    LogBlock(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::CallbackGroup> callback_group,
        const std::string block_name,
        GetDataCallback get_data_callback
    );

private:
    void m_start_log_block(const std::shared_ptr<std_msgs::msg::Int16> msg);
    void m_stop_log_block(const std::shared_ptr<std_msgs::msg::Empty> msg);

    void m_publish_log_data();
private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_timers_interface;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::string m_block_name;
    GetDataCallback m_get_data_callback;
    
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16>> m_start_log_block_sub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_stop_log_block_sub;

    
    std::shared_ptr<rclcpp::TimerBase> m_publish_log_data_timer;
    std::shared_ptr<rclcpp::Publisher<crazyflie_interfaces::msg::GenericLogData>> m_log_data_publisher;
};