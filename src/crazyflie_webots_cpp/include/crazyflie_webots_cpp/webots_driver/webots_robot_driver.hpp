#pragma once
#include <string>

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/robot.h>
#include <webots/supervisor.h>


#include <Eigen/Dense>

class WebotsRobotDriver
{
public:
    WebotsRobotDriver(const std::string robot_name, const std::string &webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip);   
    virtual ~WebotsRobotDriver();

    virtual bool step();
    double get_time_step();

    std::string get_robot_name() const { return m_robot_name; }

    Eigen::Affine3d get_robot_pose();

private:
    std::string m_robot_name;
    std::string m_webots_port;
    bool m_webots_use_tcp;
    std::string m_webots_tcp_ip;

private: 
    
    double m_time_step;
    bool m_connected;

protected:
    WbNodeRef m_robot_node;

    WbFieldRef m_position_field;
    WbFieldRef m_rotation_field;

};    