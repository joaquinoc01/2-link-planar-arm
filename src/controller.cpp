#include "controller.hpp"

JointSpaceController::JointSpaceController(const Eigen::Vector2d& Kp, const Eigen::Vector2d& Kd)
    : m_Kp{ Kp }, m_Kd{ Kd }
{
}

Eigen::Vector2d JointSpaceController::computeControl(const Eigen::Vector2d& q_des, const Eigen::Vector2d& qd_des,
                                                     const Eigen::Vector2d& q, const Eigen::Vector2d& qd)
{
    Eigen::Vector2d error = q_des - q;
    Eigen::Vector2d derror = qd_des - qd;
    return m_Kp.cwiseProduct(error) + m_Kd.cwiseProduct(derror); // Element wise product
}