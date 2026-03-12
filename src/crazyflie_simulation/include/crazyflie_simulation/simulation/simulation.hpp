#pragma once

#include "quadcopter_model/objects/quadrotor.hpp"
#include "crazyflie_simulation/simulation/IController.hpp"

#include <Eigen/Dense>


class Simulation
{
public:
    Simulation(const std::string& name, Eigen::Affine3d initial_pose = Eigen::Affine3d::Identity());
    ~Simulation();

    void update(double d_t);

    Eigen::Affine3d get_current_pose();

    void set_target_pose(const Eigen::Affine3d &target_pose);

    std::string get_name() const { return m_name; }
    double get_battery_voltage();
    double get_charge_current();
    double get_charge_state();
    bool can_fly();
    bool is_flying();
    bool is_tumbled();

private: 
    std::string m_name;
    quadcopter::Command m_cmd;
    std::shared_ptr<quadcopter::Quadrotor> m_quadrotor;

    std::unique_ptr<IController> m_controller;


    enum class ChargeState {
        BATTERY, 
        CHARGING, 
        CHARGED, 
        LOWPOWER, 
        SHUTDOWN
    };
};