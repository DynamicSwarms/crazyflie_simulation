#include "crazyflie_webots/crtp_driver/localization.hpp"

#include <Eigen/Dense>

Localization::Localization(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,   
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)
: m_logging_interface(node_logging_interface)
, m_webots_driver(webots_driver)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;

    m_pose_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::PoseStampedArray>(
        node_topics_interface,
        "/cf_positions",
        rclcpp::QoS(10),
        publisher_options);

    m_publish_timer = rclcpp::create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Localization::publish_timer_callback, this),
        m_callback_group,
        node_base_interface.get(),
        node_timers_interface.get()
    );


    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Localization initialized");
}

void
Localization::publish_timer_callback()
{
    if (auto webots_driver = m_webots_driver.lock()) {
        crazyflie_interfaces::msg::PoseStampedArray pose_array_msg;

        geometry_msgs::msg::PoseStamped pose_msg;
        Eigen::Affine3d pose = webots_driver->get_robot_pose();

        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.header.frame_id = webots_driver->get_robot_name();

        pose_msg.pose.position.x = pose.translation().x();
        pose_msg.pose.position.y = pose.translation().y();
        pose_msg.pose.position.z = pose.translation().z();

        Eigen::Quaterniond quat(pose.rotation());
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();

        pose_array_msg.poses.push_back(pose_msg);

        m_pose_publisher->publish(pose_array_msg);
    }
}