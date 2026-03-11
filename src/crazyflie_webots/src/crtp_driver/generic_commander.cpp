#include "crazyflie_webots/crtp_driver/generic_commander.hpp"
#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;

GenericCommander::GenericCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)
: m_logging_interface(node_logging_interface)
, m_webots_driver(webots_driver)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;



    m_cmd_position_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::Position>(
        node_topics_interface,
        "~/cmd_position",
        10,
        std::bind(&GenericCommander::cmd_position_callback, this, _1),
        sub_opt);

    m_notify_setpoints_stop_service = rclcpp::create_service<crazyflie_interfaces::srv::NotifySetpointsStop>(
        node_base_interface,
        node_services_interface,
        "~/notify_setpoints_stop",
        std::bind(&GenericCommander::notify_setpoints_stop_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );
     
    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Generic Commander initialized");
};



void 
GenericCommander::cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
{
    if (auto webots_driver = m_webots_driver.lock()) {
        Eigen::Vector3d target(msg->x, msg->y, msg->z);
        webots_driver->set_target(target, msg->yaw);
    }
}

void 
GenericCommander::notify_setpoints_stop_service(
    const crazyflie_interfaces::srv::NotifySetpointsStop::Request::SharedPtr request,
    crazyflie_interfaces::srv::NotifySetpointsStop::Response::SharedPtr response)
{
    (void)request;
    (void)response;
    RCLCPP_INFO(m_logging_interface->get_logger(), "Received request to stop setpoints. Webots has no notify_setpoints_stop implemented.");
}