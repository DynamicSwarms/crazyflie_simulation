#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/transition.hpp"

#include "crazyflie_simulation/crtp_driver/console.hpp"
#include "crazyflie_simulation/crtp_driver/generic_commander.hpp"
#include "crazyflie_simulation/crtp_driver/hl_commander.hpp"
#include "crazyflie_simulation/crtp_driver/localization.hpp"
#include "crazyflie_simulation/crtp_driver/logging.hpp"
#include "crazyflie_simulation/crtp_driver/parameters.hpp"

#include "crazyflie_simulation/simulation/simulation.hpp"

class Crazyflie : public rclcpp_lifecycle::LifecycleNode
{
  public: 
    Crazyflie(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("crazyflie", options)
    , p_id(declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , p_initial_position(this->declare_parameter("initial_position", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0})).get<std::vector<double>>())
    , m_simulation(std::make_shared<Simulation>("cf"+ std::to_string(p_id), Eigen::Affine3d(Eigen::Translation3d(p_initial_position[0], p_initial_position[1], p_initial_position[2]))))
    , m_console(std::make_shared<Console>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        m_simulation
    ))
    , m_generic_commander(std::make_shared<GenericCommander>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface(),
        this->get_node_logging_interface(),
        m_simulation
    ))
    , m_hl_commander(std::make_shared<HighLevelCommander>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface(),
        this->get_node_logging_interface(),
        m_simulation
    ))
    , m_localization(std::make_shared<Localization>(
        this->get_node_base_interface(),
        this->get_node_logging_interface(),
        this->get_node_topics_interface(),
        this->get_node_timers_interface(),
        m_simulation
    ))
    , m_logging(std::make_shared<Logging>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface(),
        this->get_node_logging_interface(),
        this->get_node_timers_interface(),
        this->get_node_clock_interface(),
        m_simulation
    ))
    , m_parameters(std::make_shared<Parameters>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_logging_interface(),
        this->get_node_parameters_interface(),
        m_simulation
    ))
    {    
      m_simulation_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      m_simulation_step_timer = this->create_wall_timer(
        std::chrono::milliseconds(32), // ~30Hz, matching Webots default timestep of 32ms
        std::bind(&Crazyflie::simulation_timer_callback, this),
        m_simulation_callback_group);

      m_configure_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      m_configure_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
          // Sadly this has to be done like this because when many crazyflies are started at once,
          // some of them miss the configure transition event subscription 
          // This adds an additional delay before configuring to ensure the subscription is active
          if (this->get_node_graph_interface()->count_subscribers("~/transition_event")) {
            rclcpp::sleep_for(std::chrono::milliseconds(1500));
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);//this->configure();
            m_configure_timer->cancel();  
          }
        },
        m_configure_callback_group);

      RCLCPP_INFO(get_logger(), "CrazyflieSimulation node initialized.");
    }


  void simulation_timer_callback()
  {
    if (m_simulation) {
      m_simulation->update(0.032); // Update the simulation with a fixed timestep of 32ms
    }
  }
  ~Crazyflie()
  { 

    m_simulation_step_timer->cancel();
    m_simulation_step_timer.reset();
    m_simulation.reset();
    m_console.reset();
    m_generic_commander.reset();
    m_hl_commander.reset();
    m_localization.reset();
    m_logging.reset();
    m_parameters.reset();
  } 

     /**
   * Lifecycle callbacks.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_configure(state);

    RCLCPP_DEBUG(get_logger(), "on_configure() is called.");
   
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_activate(state);
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
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
    (void)state;
    RCLCPP_DEBUG(get_logger(), "Shutting down cleanly.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  private: 
    uint8_t p_id; 
    std::vector<double> p_initial_position;

    std::shared_ptr<rclcpp::TimerBase> m_timer;
    double pos; 

    std::shared_ptr<rclcpp::CallbackGroup> m_simulation_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_simulation_step_timer;

    std::shared_ptr<rclcpp::CallbackGroup> m_configure_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_configure_timer;

    std::shared_ptr<Simulation> m_simulation;

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
  node = std::make_shared<Crazyflie>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  auto stop_spin_timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [node, &executor]() {
        if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
            executor.cancel();
            std::cerr << "Node finalized, stopping executor spin." << std::endl;
        }
    }
  );

  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}