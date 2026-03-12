#include "crazyflie_simulation/crtp_driver/hl_commander.hpp"
#include "crazyflie_simulation/crtp_driver/eigen_transformations.hpp"

#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;

HighLevelCommander::HighLevelCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<Simulation> simulation)
: m_logging_interface(node_logging_interface)
, m_simulation(simulation)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    (void)node_topics_interface;
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_land_service = rclcpp::create_service<crazyflie_interfaces::srv::Land>(
        node_base_interface,
        node_services_interface,
        "~/land",
        std::bind(&HighLevelCommander::land_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );

    m_takeoff_service = rclcpp::create_service<crazyflie_interfaces::srv::Takeoff>(
        node_base_interface,
        node_services_interface,
        "~/takeoff",
        std::bind(&HighLevelCommander::takeoff_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );

    m_goto_service = rclcpp::create_service<crazyflie_interfaces::srv::GoTo>(
        node_base_interface,
        node_services_interface,
        "~/go_to",
        std::bind(&HighLevelCommander::goto_service, this, _1, _2),
        rmw_qos_profile_services_default,
        m_callback_group
    );


    RCLCPP_DEBUG(node_logging_interface->get_logger(), "High Level Commander initialized");
};

void HighLevelCommander::land_service(
    const crazyflie_interfaces::srv::Land::Request::SharedPtr request, 
    crazyflie_interfaces::srv::Land::Response::SharedPtr response)
{
    (void)response;
    if (auto simulation = m_simulation.lock()) {
        Eigen::Affine3d current_pose = simulation->get_current_pose();
        current_pose.translation().z() = request->height + 0.02;  // add small offset to avoid ground collision

        simulation->set_target_pose(from_xyz_and_yaw(current_pose.translation(), request->yaw));
    }
}

void HighLevelCommander::takeoff_service(
    const crazyflie_interfaces::srv::Takeoff::Request::SharedPtr request, 
    crazyflie_interfaces::srv::Takeoff::Response::SharedPtr response)

{
    (void)response;
    if (auto simulation = m_simulation.lock()) {
        Eigen::Affine3d current_pose = simulation->get_current_pose();
        current_pose.translation().z() = request->height;
        simulation->set_target_pose(from_xyz_and_yaw(current_pose.translation(), request->yaw));
    }
}




void HighLevelCommander::goto_service(
    const crazyflie_interfaces::srv::GoTo::Request::SharedPtr request, 
    crazyflie_interfaces::srv::GoTo::Response::SharedPtr response)
{
    (void)response;
    if (auto simulation = m_simulation.lock()) {
        if (request->relative) {
            Eigen::Affine3d current_pose = simulation->get_current_pose();
            Eigen::Vector3d target;
            target.x() = current_pose.translation().x() + request->goal.x;
            target.y() = current_pose.translation().y() + request->goal.y;
            target.z() = current_pose.translation().z() + request->goal.z;
            double current_yaw = std::atan2(current_pose.linear()(1, 0), current_pose.linear()(0, 0)) * 180.0 / M_PI;
            double target_yaw = current_yaw + request->yaw;
            
            simulation->set_target_pose(from_xyz_and_yaw(target, target_yaw));
        } else {
            Eigen::Vector3d target_pos(request->goal.x, request->goal.y, request->goal.z);
            simulation->set_target_pose(from_xyz_and_yaw(target_pos, request->yaw));
        }
    }
}