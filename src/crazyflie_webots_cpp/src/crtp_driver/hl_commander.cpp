#include "crazyflie_webots_cpp/crtp_driver/hl_commander.hpp"

#include <Eigen/Dense>

using std::placeholders::_1;

HighLevelCommander::HighLevelCommander(
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

    m_land_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::Land>(
        node_topics_interface,
        "~/land",
        10,
        std::bind(&HighLevelCommander::land_callback, this, _1),
        sub_opt);

    m_takeoff_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::Takeoff>(
        node_topics_interface,
        "~/takeoff",
        10,
        std::bind(&HighLevelCommander::takeoff_callback, this, _1),
        sub_opt);

    m_goto_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::GoTo>(
        node_topics_interface,
        "~/go_to",
        10,
        std::bind(&HighLevelCommander::goto_callback, this, _1),
        sub_opt);

    RCLCPP_INFO(node_logging_interface->get_logger(), "High Level Commander initialized");
};

void HighLevelCommander::land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg)
{
    if (auto webots_driver = m_webots_driver.lock()) {
        Eigen::Affine3d current_pose = webots_driver->get_robot_pose();
        current_pose.translation().z() = msg->height + 0.02;  // add small offset to avoid ground collision
        webots_driver->set_target(current_pose.translation(), msg->yaw);
    }
    // HighLevelCommanderLogic::send_land(msg->height, (double)(msg->duration.sec + msg->duration.nanosec * 1e-9), msg->group_mask, msg->yaw);
}

void HighLevelCommander::takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg)
{
    if (auto webots_driver = m_webots_driver.lock()) {
        Eigen::Affine3d current_pose = webots_driver->get_robot_pose();
        current_pose.translation().z() = msg->height;
        webots_driver->set_target(current_pose.translation(), msg->yaw);
    }
    //HighLevelCommanderLogic::send_takeoff(msg->height, (double)(msg->duration.sec + msg->duration.nanosec * 1e-9), msg->group_mask, msg->yaw);
}

void HighLevelCommander::goto_callback(const crazyflie_interfaces::msg::GoTo::SharedPtr msg)
{

    if (auto webots_driver = m_webots_driver.lock()) {
        if (msg->relative) {
            Eigen::Affine3d current_pose = webots_driver->get_robot_pose();
            Eigen::Vector3d target;
            target.x() = current_pose.translation().x() + msg->goal.x;
            target.y() = current_pose.translation().y() + msg->goal.y;
            target.z() = current_pose.translation().z() + msg->goal.z;
            double current_yaw = std::atan2(current_pose.linear()(1, 0), current_pose.linear()(0, 0)) * 180.0 / M_PI;
            webots_driver->set_target(target, current_yaw + msg->yaw);
        } else {
            Eigen::Vector3d target(msg->goal.x, msg->goal.y, msg->goal.z);
            webots_driver->set_target(target, msg->yaw);
        }
    }
    //HighLevelCommanderLogic::send_go_to(msg->goal.x, msg->goal.y, msg->goal.z, msg->yaw, (double)(msg->duration.sec + msg->duration.nanosec * 1e-9), msg->relative, msg->group_mask);
}