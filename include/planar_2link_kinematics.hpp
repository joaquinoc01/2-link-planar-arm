#ifndef PLANAR_2LINK_KINEMATICS_HPP
#define PLANAR_2LINK_KINEMATICS_HPP

#include <Eigen/Dense>
#include <vector>

class Planar2LinkKinematics
{
public:
    Planar2LinkKinematics(double l1, double l2);

    Eigen::Vector2d forwardKinematics(double theta1, double theta2);
    std::vector<Eigen::Vector2d> inverseKinematics(double x, double y);
    void jacobian(double theta1, double theta2);
    bool isSingular(const Eigen::Matrix2d& jacobian);

    Eigen::Matrix2d getJacobian() const { return m_jacobian; }
    Eigen::Vector2d endEffectorVelocity(const Eigen::Vector2d& joint_vels) { return m_jacobian * joint_vels; }

private:
    double m_l1, m_l2;
    Eigen::Matrix2d m_jacobian;
};

#endif // PLANAR_2LINK_KINEMATICS_HPP