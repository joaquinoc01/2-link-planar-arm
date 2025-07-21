#include <iostream>
#include "planar_2link_kinematics.hpp"

Planar2LinkKinematics::Planar2LinkKinematics(double l1, double l2)
    : m_l1{ l1 }, m_l2{ l2 }
{
}

Eigen::Vector2d Planar2LinkKinematics::forwardKinematics(double theta1, double theta2) const
{
    Eigen::Vector2d end_effector{};

    // The end effector position in a 2 link manipulator is given by
    end_effector[0] = m_l1 * std::cos( theta1 ) + m_l2 * std::cos( theta1 + theta2 );
    end_effector[1] = m_l1 * std::sin( theta1 ) + m_l2 * std::sin( theta1 + theta2 );

    return end_effector;
}

std::vector<Eigen::Vector2d> Planar2LinkKinematics::inverseKinematics(double x, double y)
{
    std::vector<Eigen::Vector2d> solutions;

    double r_squared = x * x + y * y;
    double r = std::sqrt(r_squared);

    // Check reachability
    if (r > m_l1 + m_l2 || r < std::abs(m_l1 - m_l2)) {
        std::cerr << "Target is out of reach!" << std::endl;
        return solutions; // empty
    }

    double cos_theta2 = (r_squared - m_l1 * m_l1 - m_l2 * m_l2) / (2 * m_l1 * m_l2);

    // Clamp to avoid domain errors due to numerical issues
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);

    // Two possible values for theta2: elbow-down (+) and elbow-up (-)
    double sin_theta2_pos = std::sqrt(1 - cos_theta2 * cos_theta2);
    double sin_theta2_neg = -sin_theta2_pos;

    for (double sin_theta2 : {sin_theta2_pos, sin_theta2_neg}) {
        double theta2 = std::atan2(sin_theta2, cos_theta2);
        double k1 = m_l1 + m_l2 * cos_theta2;
        double k2 = m_l2 * sin_theta2;
        double theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        solutions.emplace_back(Eigen::Vector2d{theta1, theta2});
    }

    return solutions;
}

void Planar2LinkKinematics::jacobian(double theta1, double theta2)
{
    // The jacobian is calcualted by differentiating the forward kinematics wrt the angles
    m_jacobian(0, 0) = -m_l1 * std::sin( theta1 ) - m_l2 * std::sin( theta1 + theta2 );
    m_jacobian(0, 1) = -m_l2 * std::sin( theta1 + theta2 );
    m_jacobian(1, 0) = m_l1 * std::cos( theta1 ) + m_l2 * std::cos( theta1 + theta2 );
    m_jacobian(1, 1) = m_l2 * std::cos( theta1 + theta2 );
}

bool Planar2LinkKinematics::isSingular(const Eigen::Matrix2d& jacobian)
{
    return ((jacobian.determinant() < 1e-3) ? true : false);
}
