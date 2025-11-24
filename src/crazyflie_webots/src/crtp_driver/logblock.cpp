#include "crazyflie_webots/crtp_driver/logblock.hpp"

using std::placeholders::_1;


LogBlock::LogBlock( 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group,
    const std::string name,
    GetDataCallback get_data_callback
)
: m_base_interface(node_base_interface)
, m_logging_interface(node_logging_interface)
, m_timers_interface(node_timers_interface)
, m_callback_group(callback_group)
, m_block_name(name)
, m_get_data_callback(get_data_callback)
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    std::string ns = "~/log/" + name + "/";

    m_start_log_block_sub = rclcpp::create_subscription<std_msgs::msg::Int16>(
        node_topics_interface,
        ns + "start",
        10,
        std::bind(&LogBlock::m_start_log_block, this, _1),
        sub_opt);

    m_stop_log_block_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        ns + "stop",
        10,
        std::bind(&LogBlock::m_stop_log_block, this, _1),
        sub_opt);
    
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = callback_group;
    m_log_data_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::GenericLogData>(
        node_topics_interface,
        ns + "data",
        rclcpp::QoS(10),
        publisher_options
    );       
}

void LogBlock::m_start_log_block(const std::shared_ptr<std_msgs::msg::Int16> msg)
{
    int period_ms = msg->data * 10; // the msg data is in 0.1ms units -> a value of 100 means 1s 
    m_publish_log_data_timer = rclcpp::create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&LogBlock::m_publish_log_data, this),
        m_callback_group,
        m_base_interface.get(),
        m_timers_interface.get()
    );
    RCLCPP_INFO(m_logging_interface->get_logger(), "Starting log block with id: %d", msg->data);
}

void LogBlock::m_stop_log_block(const std::shared_ptr<std_msgs::msg::Empty> msg)
{
    m_publish_log_data_timer->cancel();
    m_publish_log_data_timer.reset();
    RCLCPP_INFO(m_logging_interface->get_logger(), "Stopping log block");
}

void LogBlock::m_publish_log_data()
{
    auto log_msg = crazyflie_interfaces::msg::GenericLogData();
    if (m_get_data_callback) {
        auto values =  m_get_data_callback(m_block_name);
        for (const auto &val : values) {
            log_msg.values.push_back(val);
        }
    }
    m_log_data_publisher->publish(log_msg);
}