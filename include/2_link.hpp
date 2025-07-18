#include <Eigen/Dense>

class Planar2LinkArm
{
public:
    Planar2LinkArm(double l1, double l2);

    Eigen::Vector2d forwardKinematics(double theta1, double theta2);
    std::vector<Eigen::Vector2d> inverseKinematics(double x, double y);
    Eigen::Matrix2d jacobian(double theta1, double theta2);
    bool isSingular(const Eigen::Matrix2d& jacobian);

private:
    double m_l1, m_l2;    
};