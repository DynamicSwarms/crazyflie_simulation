#pragma once


#include <Eigen/Dense>


class IController 
{
public:
    virtual ~IController() = default;

    void update(double d_t, Eigen::Affine3d current_pose, Eigen::Affine3d current_velocity) {
        switch (m_control_mode) {
            case ControlMode::POSITION:
                m_update_position_controller(d_t, Eigen::Affine3d(current_pose));
                break;
            case ControlMode::VELOCITY:
                m_update_velocity_controller(d_t, Eigen::Affine3d(current_velocity));
                break;
        }
    }

    virtual bool get_motor_commands(std::vector<double> &motor_commands) = 0;

    virtual bool get_velocity_commands(std::vector<double> &velocity_commands) {
        velocity_commands = m_velocity_commands;
        return false;
    };

    void set_target_pose(const Eigen::Affine3d &target_pose) 
    {
        m_desired_pose = target_pose;
        m_control_mode = ControlMode::POSITION;
    }

    void set_target_velocity_world(const Eigen::Vector3d &velocity, double yaw_rate)
    {
        m_desired_velocity = velocity;
        m_desired_yaw_rate = yaw_rate;
        m_control_mode = ControlMode::VELOCITY;
    }

private: 

    virtual void m_update_position_controller(double d_t, Eigen::Affine3d current_pose) = 0;
    virtual void m_update_velocity_controller(double d_t, Eigen::Affine3d current_velocity) = 0;

protected: 
    Eigen::Affine3d m_desired_pose = Eigen::Affine3d::Identity();
    Eigen::Vector3d m_desired_velocity = Eigen::Vector3d::Zero();
    double m_desired_yaw_rate = 0.0; // In radians per second

    enum class ControlMode {
        POSITION,
        VELOCITY
    } m_control_mode = ControlMode::POSITION;

    std::vector<double> m_motor_commands = std::vector<double>(4, 0.0); 
    std::vector<double> m_velocity_commands = std::vector<double>(6, 0.0);
};