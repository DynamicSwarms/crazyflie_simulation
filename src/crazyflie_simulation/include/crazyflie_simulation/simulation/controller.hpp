#pragma once

#include "crazyflie_simulation/simulation/IController.hpp"

class CrazyflieController : public IController
{
public: 
    CrazyflieController();
    ~CrazyflieController();

    bool get_motor_commands(std::vector<double> &motor_commands) override;

    bool get_velocity_commands(std::vector<double> &velocity_commands) override;

private:
    void m_update(double d_t, Eigen::Affine3d current_pose) override;

private: 
    // Initialize error accumulators
    double m_forward_integral = 0.0;
    double m_sideways_integral = 0.0;
    double m_up_integral = 0.0;

};