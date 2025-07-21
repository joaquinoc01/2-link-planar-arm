#include <iostream>

#include "joint_trayectory.hpp"
#include "planar_2link_kinematics.hpp"

JointTrayectory::JointTrayectory(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double duration, InterpolationType type)
    : m_q0{ start }, m_qf{ end }, m_T{ duration }, m_type{ type }
{
}

Eigen::Vector2d JointTrayectory::getPosition(double t) const
{
    double s = t / m_T;
    if (m_type == InterpolationType::Cubic)
    {
        // Cubic interpolation position: q0 + (3s² - 2s³)(qf - q0)
        return m_q0 + (3 * s * s - 2 * s * s * s) * (m_qf - m_q0);
    }
    else if (m_type == InterpolationType::Linear)
    {
        // Linear interpolation: (1 - s)q0 + sqf 
        return (1 - s) * m_q0 + s * m_qf;
    }

    return Eigen::Vector2d::Zero(); // Fallback
}

Eigen::Vector2d JointTrayectory::getVelocity(double t) const
{
    double s = t / m_T;
    if (m_type == InterpolationType::Cubic)
    {
        // Cubic interpolation velocity: (6s(1 - s)/T)(qf - q0)
        return (6 * s * (1 - s) / m_T) * (m_qf - m_q0);
    }
    else if (m_type == InterpolationType::Linear)
    {
        // Linear interpolation velocity: constant (qf - q0) / T
        return (m_qf - m_q0) / m_T;
    }

    return Eigen::Vector2d::Zero(); // Fallback
}
