#include <vector>
#include <Eigen/Dense>

#include "planar_2link_kinematics.hpp"

struct Obstacle
{
    Eigen::Vector2d center;
    double radius;
};

class Planar2LinkCollisionChecker
{
public:
    Planar2LinkCollisionChecker(Planar2LinkKinematics kinematics);

    void addObstacle(const Eigen::Vector2d& center, double radius);
    bool isNoCollision(const Eigen::Vector2d& joint_angles) const;

private:
    Planar2LinkKinematics m_kinematics;
    std::vector<Obstacle> m_obstacles;
};