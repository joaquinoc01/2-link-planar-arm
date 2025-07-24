#include "planar_2link_dynamics.hpp"

Planar2LinkDynamics::Planar2LinkDynamics(double l1, double l2, double m1, double m2,
                                         double lc1, double lc2, double I1, double I2)
    : m_l1{ l1 }, m_l2{ l2 }, m_m1{ m1 }, m_m2{ m2 }, m_lc1{ lc1 }, m_lc2{ lc2 }, m_I1{ I1 }, m_I2{ I2 }
{
    m_a = m_I1 + m_I2 + m_m2 * m_l1 * m_l1;
    m_b = m_m2 * m_l1 * m_lc2;
    m_d = m_I2 + m_m2 * m_lc2 * m_lc2;
}

Eigen::Matrix2d Planar2LinkDynamics::computeMassMatrix(const Eigen::Vector2d& q) const
{
    double c2 = std::cos(q[1]);

    Eigen:::Matrix2d M;
    M(0, 0) = m_a + 2 * m_b * c2;
    M(0, 1) = m_d + m_b * c2;
    M(1, 0) = m_d + m_b * c2;
    M(1, 1) = m_d;
    return M;
}

Eigen::Vector2d Planar2LinkDynamics::computeCoriolisVector(const Eigen::Vector2d& q, const Eigen::Vector2d& qd) const
{
    
}