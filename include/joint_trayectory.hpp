#ifndef JOINT_TRAYECTORY_HPP
#define JOINT_TRAYECTORY_HPP

#include <Eigen/Dense>

enum class InterpolationType {
    Linear,
    Cubic
};

class JointTrayectory
{
public:
    JointTrayectory(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double duration);

    Eigen::Vector2d getPosition(double t) const;
    Eigen::Vector2d getVelocity(double t) const;

private:
    Eigen::Vector2d m_q0, m_qf;
    double m_T;
    InterpolationType m_type;
};

#endif // JOINT_TRAYECTORY_HPP