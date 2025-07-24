#ifndef PLANAR_2LINK_DYNAMICS_HPP
#define PLANAR_2LINK_DYNAMICS_HPP

#include "planar_2link_kinematics.hpp"
#include <Eigen/Dense>

class Planar2LinkDynamics
{
public:
    Planar2LinkDynamics(double l1, double l2, double m1, double m2,
                        double lc1, double lc2, double I1, double I2);

    Eigen::Matrix2d computeMassMatrix(const Eigen::Vector2d& q) const;
    Eigen::Vector2d computeCoriolisVector(const Eigen::Vector2d& q, const Eigen::Vector2d& qd) const;
    Eigen::Vector2d computeGravityVector(const Eigen::Vector2d& q) const;

private:
    double m_l1, m_l2;      // Link lengths
    double m_m1, m_m2;      // Link masses
    double m_lc1, m_lc2;    // Distance to center of masses
    double m_I1, m_I2;      // Moment of inertias
    double m_a, m_b, m_d;   // Constants
};

#endif // PLANAR_2LINK_DYNAMICS_HPP