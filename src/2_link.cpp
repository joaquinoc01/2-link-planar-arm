#include <iostream>
#include "2_link.hpp"

Planar2LinkArm::Planar2LinkArm(double l1, double l2)
    : m_l1{ l1 }, m_l2{ l2 }
{
}

Eigen::Vector2d Planar2LinkArm::forwardKinematics(double theta1, double theta2)
{
    Eigen::Vector2d end_effector{};

    // The end effector position in a 2 link manipulator is given by
    end_effector[0] = m_l1 * std::cos( theta1 ) + m_l2 * std::cos( theta1 + theta2 );
    end_effector[1] = m_l1 * std::sin( theta1 ) + m_l2 * std::sin( theta1 + theta2 );

    return end_effector;
}

std::vector<Eigen::Vector2d> Planar2LinkArm::inverseKinematics(double x, double y)
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

void Planar2LinkArm::jacobian(double theta1, double theta2)
{
    // The jacobian is calcualted by differentiating the forward kinematics wrt the angles
    m_jacobian(0, 0) = -m_l1 * std::sin( theta1 ) - m_l2 * std::sin( theta1 + theta2 );
    m_jacobian(0, 1) = -m_l2 * std::sin( theta1 + theta2 );
    m_jacobian(1, 0) = m_l1 * std::cos( theta1 ) + m_l2 * std::cos( theta1 + theta2 );
    m_jacobian(1, 1) = m_l2 * std::cos( theta1 + theta2 );
}

bool Planar2LinkArm::isSingular(const Eigen::Matrix2d& jacobian)
{
    return ((jacobian.determinant() < 1e-3) ? true : false);
}

int main()
{
    Planar2LinkArm arm(1.0, 1.0); // both links = 1m
    double theta1 = M_PI / 4.0;   // 45 degrees
    double theta2 = M_PI / 4.0;   // 45 degrees

    Eigen::Vector2d pos = arm.forwardKinematics(theta1, theta2);
    std::cout << "End-effector position: " << pos.transpose() << std::endl;

    auto ik_solutions = arm.inverseKinematics(1.0, 1.0);
    for (const auto& sol : ik_solutions) {
        double theta1_deg = sol[0] * 180.0 / M_PI;
        double theta2_deg = sol[1] * 180.0 / M_PI;
        std::cout << "Theta1: " << theta1_deg << " deg, "
                  << "Theta2: " << theta2_deg << " deg" << std::endl;
    }

    // Update the internal jacobian matrix inside the object
    arm.jacobian(theta1, theta2);

    // Access the updated jacobian via getter
    Eigen::Matrix2d J = arm.getJacobian();
    std::cout << "Jacobian:\n" << J << std::endl;

    if (arm.isSingular(J))
        std::cout << "Warning: Jacobian is close to singular!" << std::endl;
    else
        std::cout << "Jacobian is non-singular." << std::endl;

    Eigen::Vector2d joint_velocities{0.1, 0.2};  // radians per second for joint1 and joint2
    Eigen::Vector2d ee_velocity = arm.endEffectorVelocity(joint_velocities);

    std::cout << "End-effector velocity: " << ee_velocity.transpose() << " m/s" << std::endl;

    return 0;
}
