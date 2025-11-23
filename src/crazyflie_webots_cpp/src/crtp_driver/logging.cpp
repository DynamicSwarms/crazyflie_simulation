#include "crazyflie_webots_cpp/crtp_driver/logging.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

#define STATE_BLOCK_ID 0
#define POSE_BLOCK_ID 1

Logging::Logging( 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)
: m_logging_interface(node_logging_interface)
, m_webots_driver(webots_driver)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_downdload_toc_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/download_logging_toc",
        10,
        std::bind(&Logging::download_toc_callback, this, _1),
        sub_opt);

    m_get_toc_info_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/get_logging_toc_info",
        10,
        std::bind(&Logging::get_toc_info_callback, this, _1),
        sub_opt);
    
    m_publish_state_timer = rclcpp::create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Logging::publish_state_timer_callback, this),
        m_callback_group,
        node_base_interface.get(),
        node_timers_interface.get()
    );

    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;

    m_state_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::GenericLogData>(
        node_topics_interface,
        "~/state",
        rclcpp::QoS(10),
        publisher_options
    );

    RCLCPP_INFO(node_logging_interface->get_logger(), "Logging  initialized");
}



void Logging::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(m_logging_interface->get_logger(), "Downloading logging TOC");
}

void Logging::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(m_logging_interface->get_logger(), "Getting logging TOC info");
}

void
Logging::publish_state_timer_callback()
{
    if (auto webots_driver = m_webots_driver.lock()) {
        crazyflie_interfaces::msg::GenericLogData log_msg;
        log_msg.values.push_back(webots_driver->get_battery_voltage());
        log_msg.values.push_back(webots_driver->get_charge_current());
        log_msg.values.push_back(webots_driver->get_charge_state());
        log_msg.values.push_back((double)webots_driver->can_fly());
        log_msg.values.push_back((double)webots_driver->is_flying());
        log_msg.values.push_back((double)webots_driver->is_tumbled());
        m_state_publisher->publish(log_msg);
    }
}