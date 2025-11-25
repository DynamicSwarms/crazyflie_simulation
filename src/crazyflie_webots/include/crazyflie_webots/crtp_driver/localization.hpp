

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"

#include "crazyflie_interfaces/msg/pose_stamped_array.hpp"

class Localization {
public:
    Localization(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,   
        std::shared_ptr<WebotsCrazyflieDriver> webots_driver
    );
private: 
    void publish_timer_callback();

private:
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;
    
    std::weak_ptr<WebotsCrazyflieDriver> m_webots_driver;
    
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::TimerBase> m_publish_timer;
    std::shared_ptr<rclcpp::Publisher<crazyflie_interfaces::msg::PoseStampedArray>> m_pose_publisher;
};