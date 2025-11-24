#include "crazyflie_webots_cpp/crtp_driver/generic_commander.hpp"
#include <Eigen/Dense>

using std::placeholders::_1;

GenericCommander::GenericCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)
: m_logging_interface(node_logging_interface)
, m_webots_driver(webots_driver)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_notify_setpoints_stop_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::NotifySetpointsStop>(
        node_topics_interface,
        "~/notify_setpoints_stop",
        rclcpp::QoS(10),
        std::bind(&GenericCommander::notify_setpoints_stop_callback, this, _1),
        sub_opt);

    m_cmd_position_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::Position>(
        node_topics_interface,
        "~/cmd_position",
        10,
        std::bind(&GenericCommander::cmd_position_callback, this, _1),
        sub_opt);
        
    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Generic Commander initialized");
};

void GenericCommander::notify_setpoints_stop_callback(const crazyflie_interfaces::msg::NotifySetpointsStop::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(m_logging_interface->get_logger(), "Webots has no notify_setpoints_stop implemented.");
}

void GenericCommander::cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
{
    if (auto webots_driver = m_webots_driver.lock()) {
        Eigen::Vector3d target(msg->x, msg->y, msg->z);
        webots_driver->set_target(target, msg->yaw);
    }
}