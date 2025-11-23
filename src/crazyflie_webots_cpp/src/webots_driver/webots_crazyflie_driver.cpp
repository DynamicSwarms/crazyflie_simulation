#include "crazyflie_webots_cpp/webots_driver/webots_crazyflie_driver.hpp"
#include <cstdio>
#include <iostream>

WebotsCrazyflieDriver::WebotsCrazyflieDriver(int id, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip)
      : WebotsRobotDriver("cf" + std::to_string(id), webots_port, webots_use_tcp, webots_tcp_ip) 
      ,m_id(id)
{
    // Initialize motors
    m_m1_motor = wb_robot_get_device("m1_motor");
    wb_motor_set_position(m_m1_motor, INFINITY);
    m_m2_motor = wb_robot_get_device("m2_motor");
    wb_motor_set_position(m_m2_motor, INFINITY);
    m_m3_motor = wb_robot_get_device("m3_motor");
    wb_motor_set_position(m_m3_motor, INFINITY);
    m_m4_motor = wb_robot_get_device("m4_motor");
    wb_motor_set_position(m_m4_motor, INFINITY);
  
    // Initialize sensors
    m_gps = wb_robot_get_device("gps");
    wb_gps_enable(m_gps, static_cast<int>(get_time_step()));
    m_zranger = wb_robot_get_device("range");
    wb_distance_sensor_enable(m_zranger, static_cast<int>(get_time_step()));

    Eigen::Vector3d pos = get_robot_pose().translation();
    m_target = {pos.x(), pos.y(), pos.z()};
    
    m_past_time = wb_robot_get_time();
}


bool
WebotsCrazyflieDriver::step()
{

    // Controller of Crazyflie in Webots using a PI controller to reach target position
    // Step the simulation and then read the sensors, compute control and send actuators commands
    if (!WebotsRobotDriver::step()) return false;
    return true;

    double Kp = 1.0; // Proportional gain
    double Ki = 0.1; // Integral gain
    
    const double dt = wb_robot_get_time() - m_past_time;
    // Get measurements
    Eigen::Vector3d pos = get_robot_pose().translation();
    double x_global = pos.x();
    double y_global = pos.y();
    double z_global = pos.z();
    
    // Get target 
    double target_x = m_target[0];
    double target_y = m_target[1];
    double target_z = m_target[2];

    // Calculate Error
    double forward_error = target_x - x_global;
    double sideways_error = target_y - y_global;
    double up_error = target_z - z_global;

    // Calculate integral
    m_forward_integral += forward_error * dt;
    m_sideways_integral += sideways_error * dt;
    m_up_integral += up_error * dt;
        
    // Avoid going lower than the floor
    if (z_global < 0.025 && target_z <= 0.025) {
     up_error = 0;
     m_up_integral = 0; // Avoid windup due to boundry condition
    }
          
    // Calculate PI controller output
    double forward_desired = Kp * forward_error + Ki * m_forward_integral;
    double sideways_desired = Kp * sideways_error + Ki * m_sideways_integral;
    double up_desired = Kp * up_error + Ki * m_up_integral;
    
    const double vel[6] = {forward_desired, sideways_desired, up_desired, 0, 0,0};
    wb_supervisor_node_set_velocity(m_robot_node, vel);
    const double rot[4] = {0.0, 0.0, 1.0, m_target_yaw_deg * M_PI / 180.0};
    wb_supervisor_field_set_sf_rotation(m_rotation_field, rot);
    
    // Setting motorspeed for nicer visuals
    int motor_speed = 48;
    if (target_z < 0.025 && up_error <= 0) motor_speed = 0;
    // Shut up motors if on the ground
    wb_motor_set_velocity(m_m1_motor, -motor_speed);
    wb_motor_set_velocity(m_m2_motor, motor_speed);
    wb_motor_set_velocity(m_m3_motor, -motor_speed);
    wb_motor_set_velocity(m_m4_motor, motor_speed);
      
    // Publish zranger values
    double z_range_value = wb_distance_sensor_get_value(m_zranger);

    m_past_time = wb_robot_get_time();
    return true;
}

void
WebotsCrazyflieDriver::set_target(Eigen::Vector3d target, double yaw_deg)
{
    m_target = std::vector<double> {target.x(), target.y(), target.z()};
    m_target_yaw_deg = yaw_deg;

}

double
WebotsCrazyflieDriver::get_battery_voltage()
{
    return 4.2;    // Dummy value for battery voltage
}

double
WebotsCrazyflieDriver::get_charge_current()
{
    return 0.0;    // Dummy value for charge current
}

double
WebotsCrazyflieDriver::get_charge_state()
{
    return double(ChargeState::CHARGED); // Dummy value for charge state
} 

bool
WebotsCrazyflieDriver::can_fly()
{
    return true; // Dummy value for can fly
}

bool
WebotsCrazyflieDriver::is_flying()
{
    double z_global = get_robot_pose().translation().z();
    return (z_global > 0.05); // Consider flying if above 5 cm
}

bool
WebotsCrazyflieDriver::is_tumbled()
{
    return false; // Dummy value for is tumbled
}
