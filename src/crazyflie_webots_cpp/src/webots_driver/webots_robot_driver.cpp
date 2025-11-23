#include "crazyflie_webots_cpp/webots_driver/webots_robot_driver.hpp"
#include <iostream>

WebotsRobotDriver::WebotsRobotDriver(const std::string robot_name, const std::string &webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip)
      : m_robot_name(robot_name)
      , m_webots_port(webots_port)
      , m_webots_use_tcp(webots_use_tcp)
      , m_webots_tcp_ip(webots_tcp_ip)
{
    std::string url;
    if (m_webots_use_tcp) {
        url = "tcp://" + m_webots_tcp_ip + ":" + m_webots_port + "/" + robot_name;
    } else {
        url = "ipc://" + m_webots_port + "/" + robot_name;
    }
    setenv("WEBOTS_CONTROLLER_URL", url.c_str(), 1);

    wb_robot_init();
    m_connected = true;
    m_time_step = wb_robot_get_basic_time_step();

    m_robot_node = wb_supervisor_node_get_self();
    m_position_field = wb_supervisor_node_get_field(m_robot_node, "translation");
    m_rotation_field = wb_supervisor_node_get_field(m_robot_node, "rotation");

    std::cerr << "Connected to robot in simulation: " << robot_name << std::endl;    
}

WebotsRobotDriver::~WebotsRobotDriver()
{
    if (m_connected) wb_robot_cleanup();
}

bool
WebotsRobotDriver::step()
{
    if (m_connected && wb_robot_step(m_time_step) == -1)
    {
        m_connected = false;
        return false;
    }
    return true;
}

double
WebotsRobotDriver::get_time_step()
{
    return m_time_step;
}

Eigen::Affine3d
WebotsRobotDriver::get_robot_pose()
{
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    const double *position = wb_supervisor_field_get_sf_vec3f(m_position_field);
    const double *rotation = wb_supervisor_field_get_sf_rotation(m_rotation_field);
    pose.translation() << position[0], position[1], position[2];
    Eigen::AngleAxisd angle_axis(rotation[3], Eigen::Vector3d(rotation[0], rotation[1], rotation[2]));
    pose.linear() = angle_axis.toRotationMatrix();
    return pose;
}