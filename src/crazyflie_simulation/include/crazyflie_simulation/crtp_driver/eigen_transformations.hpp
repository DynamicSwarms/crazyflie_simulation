#pragma once
#include <Eigen/Dense>



inline Eigen::Affine3d
from_xyz_and_yaw(const Eigen::Vector3d &xyz, double yaw, bool yaw_deg = false)
{
    double yaw_rad = yaw_deg ? yaw * M_PI / 180.0 : yaw;
    Eigen::AngleAxisd yaw_rot(yaw_rad, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = xyz;
    pose.linear() = yaw_rot.toRotationMatrix();
    return pose;
}