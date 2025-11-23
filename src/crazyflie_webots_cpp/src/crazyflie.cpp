#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "crazyflie_webots_cpp/crtp_driver/console.hpp"
#include "crazyflie_webots_cpp/crtp_driver/generic_commander.hpp"
#include "crazyflie_webots_cpp/crtp_driver/hl_commander.hpp"
#include "crazyflie_webots_cpp/crtp_driver/localization.hpp"
#include "crazyflie_webots_cpp/crtp_driver/logging.hpp"
#include "crazyflie_webots_cpp/crtp_driver/parameters.hpp"

#include "crazyflie_webots_cpp/webots_driver/webots_crazyflie_driver.hpp"


class Crazyflie : public rclcpp_lifecycle::LifecycleNode
{
  public: 
    Crazyflie(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("crazyflie", options)
    , m_id(declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_webots_port(declare_parameter("webots_port", rclcpp::ParameterValue(1234), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_webots_use_tcp(declare_parameter("webots_use_tcp", rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<bool>())
    , m_webots_tcp_ip(declare_parameter("webots_tcp_ip", rclcpp::ParameterValue("127.0.0.1"), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>())
    , m_wb_driver(std::make_shared<WebotsCrazyflieDriver>(m_id, m_webots_port, m_webots_use_tcp, m_webots_tcp_ip))
    , m_console(std::make_shared<Console>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        m_wb_driver
    ))
    , m_generic_commander(std::make_shared<GenericCommander>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        m_wb_driver
    ))
    , m_hl_commander(std::make_shared<HighLevelCommander>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        m_wb_driver
    ))
    , m_localization(std::make_shared<Localization>(
        this->get_node_base_interface(),
        this->get_node_logging_interface(),
        this->get_node_topics_interface(),
        this->get_node_timers_interface(),
        m_wb_driver
    ))
    , m_logging(std::make_shared<Logging>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        this->get_node_timers_interface(),
        m_wb_driver
    ))
    , m_parameters(std::make_shared<Parameters>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        this->get_node_parameters_interface(),
        m_wb_driver
    ))
    {    
      m_webots_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      m_webots_step_timer = this->create_wall_timer(
        std::chrono::microseconds(static_cast<long long>(m_wb_driver->get_time_step() * 1000.0)),
        std::bind(&Crazyflie::webots_step_timer_callback, this),
        m_webots_callback_group);

        std::cerr << "Initialized CrazyflieWebotsDriver." << std::endl;
    }


  void webots_step_timer_callback()
  {
    if (m_wb_driver) {
      if (!m_wb_driver->step()) {

        m_wb_driver.reset();
        this->shutdown();
        
      }
    }
  }
  ~Crazyflie()
  { 

    std::cerr << "Cleaning up CrazyflieWebotsDriver." << std::endl;
    m_webots_step_timer->cancel();
    m_webots_step_timer.reset();
    std::cerr << "Cleaning up WebotsCrazyflieDriver." << std::endl;
    m_wb_driver.reset();
    std::cerr << "Cleaning up CRTP modules." << std::endl;
    m_console.reset();
    std::cerr << "Cleaning up GenericCommander." << std::endl;
    //m_generic_commander.reset();
    std::cerr << "Cleaning up HighLevelCommander." << std::endl;
    m_hl_commander.reset();
    std::cerr << "Cleaning up Localization." << std::endl;
    m_localization.reset();
    std::cerr << "Cleaning up Logging." << std::endl;
    m_logging.reset();
    std::cerr << "Cleaning up Parameters." << std::endl;
    m_parameters.reset();
    std::cerr << "Cleaned up CrazyflieWebotsDriver." << std::endl;
  } 

     /**
   * Lifecycle callbacks.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    //if (this->init())
    //{
    //  RCLCPP_INFO(get_logger(), "Successfully configured!");
    //  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    //}
    //else
    //{
    //  RCLCPP_DEBUG(get_logger(), "Configuring failed!");
    //  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    //}
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_activate(state);
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_deactivate(state);
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_DEBUG(get_logger(), "Shutting down cleanly.");
    std::cerr << "Shutting down cleanly." << std::endl; 
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  private: 
    uint8_t m_id; 

    int m_webots_port;
    bool m_webots_use_tcp; 
    std::string m_webots_tcp_ip; 

    std::shared_ptr<rclcpp::TimerBase> m_timer;
    double pos; 

    std::shared_ptr<rclcpp::CallbackGroup> m_webots_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_webots_step_timer;

    std::shared_ptr<WebotsCrazyflieDriver> m_wb_driver;

    std::shared_ptr<Console> m_console;
    std::shared_ptr<GenericCommander> m_generic_commander;
    std::shared_ptr<HighLevelCommander> m_hl_commander;
    std::shared_ptr<Localization> m_localization;
    std::shared_ptr<Logging> m_logging;
    std::shared_ptr<Parameters> m_parameters;
};

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<Crazyflie> node;
  try {
    node = std::make_shared<Crazyflie>(options);
  } catch (const WebotsInitException &e) {
    std::cerr << "Failed to initialize Crazyflie node: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  while (rclcpp::ok() &&  !(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)) executor.spin_some();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();

  std::cerr << "Crazyflie node shut down cleanly." << std::endl;
  return 0;
}

//export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH
//ros2 run --prefix "$WEBOTS_HOME/webots-controller --robot-name=cf1" webots_cpp test_driver 