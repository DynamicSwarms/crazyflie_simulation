#include "crazyflie_simulation/crazyflie.hpp"

#include <cstdio>
#include <iostream>
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp_lifecycle/transition.hpp"

Crazyflie::Crazyflie(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("crazyflie", options)
    , p_id(declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , p_initial_position(this->declare_parameter("initial_position", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0})).get<std::vector<double>>())
    , p_publish_to_cf(this->declare_parameter("publish_to_cf", rclcpp::ParameterValue(true)).get<bool>())
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
        this->get_node_clock_interface(),
        m_simulation,
        p_publish_to_cf
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
      m_simulation_step_timer = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&Crazyflie::simulation_timer_callback, this),
        m_simulation_callback_group);

      m_configure_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      m_configure_timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
          this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
          m_configure_timer->cancel();  
        },
        m_configure_callback_group);

      RCLCPP_INFO(get_logger(), "CrazyflieSimulation node initialized.");
    }




Crazyflie::~Crazyflie()
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

Eigen::Affine3d 
Crazyflie::get_pose() const
{
  if (m_simulation) {
    return m_simulation->get_current_pose();
  } else {
    return Eigen::Affine3d::Identity();
  }
}

void Crazyflie::simulation_timer_callback()
{
  if (m_simulation) {
    m_simulation->update(0.1); // Update the simulation with a fixed timestep of 32ms
  }
}

/**
 * Lifecycle callbacks.
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Crazyflie::on_configure(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_configure(state);

  RCLCPP_DEBUG(get_logger(), "on_configure() is called.");
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Crazyflie::on_activate(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_activate(state);
  RCLCPP_INFO(get_logger(), "on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Crazyflie::on_deactivate(const rclcpp_lifecycle::State &state)
{
  LifecycleNode::on_deactivate(state);
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Crazyflie::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Crazyflie::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "Shutting down cleanly.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Crazyflie)