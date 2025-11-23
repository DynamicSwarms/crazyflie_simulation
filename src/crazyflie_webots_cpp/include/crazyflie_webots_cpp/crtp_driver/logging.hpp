

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_webots_cpp/webots_driver/webots_crazyflie_driver.hpp"

#include "crazyflie_interfaces/msg/generic_log_data.hpp"

#include "std_msgs/msg/empty.hpp"

class Logging{
public:
    Logging(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<WebotsCrazyflieDriver> webots_driver
    );

private: 

    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void publish_state_timer_callback();

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> m_topics_interface;

    std::weak_ptr<WebotsCrazyflieDriver> m_webots_driver;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_downdload_toc_sub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_get_toc_info_sub;

    std::shared_ptr<rclcpp::TimerBase> m_publish_state_timer;
    std::shared_ptr<rclcpp::Publisher<crazyflie_interfaces::msg::GenericLogData>> m_state_publisher;
};  