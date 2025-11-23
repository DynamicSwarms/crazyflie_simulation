

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_webots_cpp/webots_driver/webots_crazyflie_driver.hpp"

#include "crazyflie_interfaces/msg/takeoff.hpp"
#include "crazyflie_interfaces/msg/land.hpp"
#include "crazyflie_interfaces/msg/go_to.hpp"


class HighLevelCommander {
public:
    HighLevelCommander(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<WebotsCrazyflieDriver> webots_driver
    );

private: 

    void land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg);
    void takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg);
    void goto_callback(const crazyflie_interfaces::msg::GoTo::SharedPtr msg);
    

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;

    std::weak_ptr<WebotsCrazyflieDriver> m_webots_driver;
    
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::Subscription<crazyflie_interfaces::msg::Land>> m_land_sub;
    std::shared_ptr<rclcpp::Subscription<crazyflie_interfaces::msg::Takeoff>> m_takeoff_sub;
    std::shared_ptr<rclcpp::Subscription<crazyflie_interfaces::msg::GoTo>> m_goto_sub;
};