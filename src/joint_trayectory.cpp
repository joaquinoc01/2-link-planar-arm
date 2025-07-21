#include <iostream>

#include "joint_trayectory.hpp"
#include "planar_2link_kinematics.hpp"

JointTrayectory::JointTrayectory(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double duration)
    : m_q0{ start }, m_qf{ end }, m_T{ duration }
{
}

Eigen::Vector2d JointTrayectory::getPosition(double t) const
{
    // Linear interpolation position = (1 - s)q0 + sqf 
    return ((1 - t / m_T) * m_q0 + t / m_T * m_qf);
}

Eigen::Vector2d JointTrayectory::getVelocity(double t) const
{
    // Linear interpolation velocity = (qf - q0) / T
    return ( m_qf - m_q0 ) / m_T;
}