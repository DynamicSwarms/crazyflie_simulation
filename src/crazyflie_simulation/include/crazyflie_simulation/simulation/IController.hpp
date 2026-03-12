#pragma once


#include <Eigen/Dense>


class IController 
{
public:
    virtual ~IController() = default;

    void update(double d_t, Eigen::Affine3d current_pose) {
        m_update(d_t, Eigen::Affine3d(current_pose));
    }

    virtual bool get_motor_commands(std::vector<double> &motor_commands) = 0;

    virtual bool get_velocity_commands(std::vector<double> &velocity_commands) {
        velocity_commands = m_velocity_commands;
        return false;
    };

    void set_target_pose(const Eigen::Affine3d &target_pose) {
        m_desired_pose = target_pose;
    }

private: 

    virtual void m_update(double d_t, Eigen::Affine3d current_pose) = 0;

protected: 
    Eigen::Affine3d m_desired_pose = Eigen::Affine3d::Identity();
    std::vector<double> m_motor_commands = std::vector<double>(4, 0.0); 
    std::vector<double> m_velocity_commands = std::vector<double>(6, 0.0);
};