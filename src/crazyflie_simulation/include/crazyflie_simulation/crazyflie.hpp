#pragma once

#include <memory>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
  explicit Crazyflie(const rclcpp::NodeOptions & options);
  ~Crazyflie() override;

  Eigen::Affine3d get_pose() const;

private:
  void simulation_timer_callback();

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  uint8_t p_id{};
  std::vector<double> p_initial_position;
  bool p_publish_to_cf{true};

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