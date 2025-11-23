#pragma once


#include <rclcpp/rclcpp.hpp>

#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
class CrazyflieLifecycleClient 
{
    using DisconnectCallback = std::function<void(int id)>;

    public: 
    CrazyflieLifecycleClient(
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        rclcpp::CallbackGroup::SharedPtr callback_group,
        int id, 
        DisconnectCallback disconnect_callback);
    
    ~CrazyflieLifecycleClient();

    bool activate_crazyflie_sync();
    void deactivate_crazyflie_async();

    bool shutdown_crazyflie_sync(std::optional<std::chrono::milliseconds> timeout_ms);
    void shutdown_crazyflie_async();

    bool transition_crazyflie_sync(uint8_t id, const std::string &label, std::optional<std::chrono::milliseconds> timeout_ms);

    std::future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>>
    transition_crazyflie_async(uint8_t id, const std::string &label);


private: 
    void m_transition_event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

private: 
    int m_id;

    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> m_transition_event_sub;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_change_state_client;

    DisconnectCallback m_disconnect_callback;
};