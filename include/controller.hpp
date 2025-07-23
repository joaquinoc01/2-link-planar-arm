#include <Eigen/Dense>

class JointSpaceController
{
public:
    JointSpaceController(const Eigen::Vector2d& Kp, const Eigen::Vector2d& Kd);

    Eigen::Vector2d computeControl(const Eigen::Vector2d& q_des, const Eigen::Vector2d& qd_des,
                                   const Eigen::Vector2d& q, const Eigen::Vector2d& qd);

private:
    Eigen::Vector2d m_Kp;
    Eigen::Vector2d m_Kd;
};