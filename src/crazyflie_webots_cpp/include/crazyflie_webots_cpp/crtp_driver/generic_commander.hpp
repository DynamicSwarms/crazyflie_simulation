

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_webots_cpp/webots_driver/webots_crazyflie_driver.hpp"

#include "crazyflie_interfaces/msg/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/msg/position.hpp"


class GenericCommander  {
public:
    GenericCommander(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<WebotsCrazyflieDriver> webots_driver
    );

private: 

    void notify_setpoints_stop_callback(const crazyflie_interfaces::msg::NotifySetpointsStop::SharedPtr msg);
    void cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg);
    

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;

    std::weak_ptr<WebotsCrazyflieDriver> m_webots_driver;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group; 

    std::shared_ptr<rclcpp::Subscription<crazyflie_interfaces::msg::NotifySetpointsStop>> m_notify_setpoints_stop_sub;
    std::shared_ptr<rclcpp::Subscription<crazyflie_interfaces::msg::Position>> m_cmd_position_sub;};