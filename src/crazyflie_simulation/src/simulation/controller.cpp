#include "crazyflie_simulation/simulation/controller.hpp"

CrazyflieController::CrazyflieController()
{
}

CrazyflieController::~CrazyflieController()
{
}
#include <iostream>
void 
CrazyflieController::m_update(double d_t, Eigen::Affine3d current_pose)
{
    double Kp = 1.0; // Proportional gain
    double Ki = 0.1; // Integral gain
    
    // Get measurements
    double x_global = current_pose.translation().x();
    double y_global = current_pose.translation().y();
    double z_global = current_pose.translation().z();
    
    // Get target 
    double target_x = m_desired_pose.translation().x();
    double target_y = m_desired_pose.translation().y();
    double target_z = m_desired_pose.translation().z();

    // Calculate Error
    double forward_error = target_x - x_global;
    double sideways_error = target_y - y_global;
    double up_error = target_z - z_global;

    // Calculate integral
    m_forward_integral += forward_error * d_t;
    m_sideways_integral += sideways_error * d_t;
    m_up_integral += up_error * d_t;
        
    // Avoid going lower than the floor
    if (z_global < -0.1 && target_z <= -0.1) {
     up_error = 0;
     m_up_integral = 0; // Avoid windup due to boundry condition
    }
    std::cerr << target_z << " " << z_global << " " << up_error << " " << m_desired_pose.translation().transpose() << std::endl;
          
    // Calculate PI controller output
    double forward_desired = Kp * forward_error + Ki * m_forward_integral;
    double sideways_desired = Kp * sideways_error + Ki * m_sideways_integral;
    double up_desired = Kp * up_error + Ki * m_up_integral;
    
    m_velocity_commands[0] = forward_desired;
    m_velocity_commands[1] = sideways_desired;
    m_velocity_commands[2] = up_desired;

    m_velocity_commands[3] = 0.0; // No rotation around x-axis
    m_velocity_commands[4] = 0.0; // No rotation around y-axis

    // Simple P controller for yaw (z-rotation)
    double Kyaw = 0.8; // Proportional gain for yaw

    double current_yaw = std::atan2(current_pose.rotation()(1, 0), current_pose.rotation()(0, 0));

    double target_yaw = std::atan2(m_desired_pose.rotation()(1, 0), m_desired_pose.rotation()(0, 0));

    double yaw_error = target_yaw - current_yaw;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error)); // Wrap to [-pi, pi]
    double yaw_desired_vel = Kyaw * yaw_error;
    m_velocity_commands[5] = yaw_desired_vel; // Rotation around z-axis (yaw)    
}

bool
CrazyflieController::get_motor_commands(std::vector<double> &motor_commands)
{
    motor_commands = m_motor_commands;
    return false;
}

bool 
CrazyflieController::get_velocity_commands(std::vector<double> &velocity_commands)
{
    velocity_commands = m_velocity_commands;
    return true;
}