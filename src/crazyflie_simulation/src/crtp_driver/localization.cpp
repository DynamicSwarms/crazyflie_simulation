#include "crazyflie_simulation/crtp_driver/localization.hpp"

#include <Eigen/Dense>

Localization::Localization(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,   
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<Simulation> simulation,
    bool publish_to_cf)
: m_logging_interface(node_logging_interface)
, m_simulation(simulation)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;

    if (publish_to_cf) {
        m_pose_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::PoseNamedArray>(
            node_topics_interface,
            "/cf_positions",
            rclcpp::QoS(10),
            publisher_options);

        m_publish_timer = rclcpp::create_timer(
            node_base_interface,
            node_timers_interface,
            node_clock_interface->get_clock(),
            std::chrono::milliseconds(50),
            std::bind(&Localization::publish_timer_callback, this),
            m_callback_group
        );
    }

    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Localization initialized");
}

void
Localization::publish_timer_callback()
{
    if (auto simulation = m_simulation.lock()) {
        crazyflie_interfaces::msg::PoseNamedArray pose_array_msg;
        pose_array_msg.header.frame_id = "world";

        crazyflie_interfaces::msg::PoseNamed pose_msg;
        Eigen::Affine3d pose = simulation->get_current_pose();

        pose_msg.header.stamp = rclcpp::Clock().now();
        pose_msg.name = simulation->get_name();
        pose_msg.rotation_valid = true;

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