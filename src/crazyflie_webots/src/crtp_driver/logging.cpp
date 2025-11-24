#include "crazyflie_webots/crtp_driver/logging.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Dense>


using std::placeholders::_1;

#define STATE_BLOCK_ID 0
#define POSE_BLOCK_ID 1

Logging::Logging( 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)

: m_base_interface(node_base_interface)
, m_topics_interface(node_topics_interface)
, m_logging_interface(node_logging_interface)
, m_timers_interface(node_timers_interface)
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

    m_create_log_block_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::LogBlock>(
        node_topics_interface,
        "~/create_log_block",
        10,
        std::bind(&Logging::m_create_log_block, this, _1),
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

    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Logging  initialized");
}



void Logging::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(m_logging_interface->get_logger(), "Downloading logging TOC");
}

void Logging::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(m_logging_interface->get_logger(), "Getting logging TOC info");
}

void Logging::m_create_log_block(const crazyflie_interfaces::msg::LogBlock::SharedPtr msg)
{
    std::vector<std::string> variables = msg->variables;
    std::string name = msg->name;

    if (m_log_blocks.count(name)) {
        RCLCPP_INFO(m_logging_interface->get_logger(), "Log block with name '%s' already exists.", name.c_str());
        return;
    }

    auto log_block = std::make_shared<LogBlock>(
        m_base_interface,
        m_topics_interface,
        m_logging_interface,
        m_timers_interface,
        m_callback_group,
        name,
        std::bind(&Logging::m_get_data_callback, this, std::placeholders::_1)
    );
    m_log_blocks[name] = log_block;
    m_log_block_variables[name] = variables;

    RCLCPP_INFO(m_logging_interface->get_logger(), "Creating log block with name: %s", msg->name.c_str());
}
std::vector<double>  
Logging::m_get_data_callback(const std::string &block_name)
{
    std::vector<double> data;
    if (auto webots_driver = m_webots_driver.lock()) {
        if (!m_log_block_variables.count(block_name)) {
            RCLCPP_WARN(m_logging_interface->get_logger(), "Requested data for unknown log block: %s", block_name.c_str());
            return data;
        }

        Eigen::Affine3d pose = webots_driver->get_robot_pose();
        Eigen::Quaterniond q(pose.rotation());

        std::vector<std::string> variables = m_log_block_variables[block_name];
        for (const auto &var : variables) {
            
            if (var == "range.front") {
                data.push_back(webots_driver->get_range_front());
            } else if (var == "range.back") {
                data.push_back(webots_driver->get_range_back());
            } else if (var == "range.up") {
                data.push_back(webots_driver->get_range_up());
            } else  if (var == "range.left") {
                data.push_back(webots_driver->get_range_left());
            } else  if (var == "range.right") {
                data.push_back(webots_driver->get_range_right());
            } else if (var == "range.zrange") {
                data.push_back(webots_driver->get_range_zrange());
            } else if (var == "stateEstimate.x") {
                data.push_back(pose.translation().x());
            } else if (var == "stateEstimate.y") {
                data.push_back(pose.translation().y());
            } else if (var == "stateEstimate.z") {
                data.push_back(pose.translation().z());
            } else if (var == "stateEstimate.yaw") {
                // Extract yaw from rotation matrix
                Eigen::Matrix3d rotation = pose.rotation();
                double yaw = atan2(rotation(1,0), rotation(0,0));
                data.push_back(yaw);
            } else if (var == "stateEstimate.qx") {
                data.push_back(q.x());
            } else if (var == "stateEstimate.qy") {
                data.push_back(q.y());
            } else if (var == "stateEstimate.qz") {
                data.push_back(q.z());
            } else if (var == "stateEstimate.qw") {
                data.push_back(q.w());
            } else if (var == "pm.vbat") {
                data.push_back(webots_driver->get_battery_voltage());
            } else if (var == "pm.chargeCurrent") {
                data.push_back(webots_driver->get_charge_current());
            } else if (var == "pm.state") {
                data.push_back(webots_driver->get_charge_state());
            } else {
                RCLCPP_WARN(m_logging_interface->get_logger(), "Unknown variable requested: %s", var.c_str());
                data.push_back(0.0);
            }
        }
    }
    return data;
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