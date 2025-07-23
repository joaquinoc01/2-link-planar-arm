#include <iostream>

#include "collision_checker.hpp"

Planar2LinkCollisionChecker::Planar2LinkCollisionChecker(double l1, double l2)
    : m_l1{ l1 }, m_l2{ l2 }
{
}

void Planar2LinkCollisionChecker::addObstacle(const Eigen::Vector2d& center, double radius)
{
    Obstacle obs;
    obs.center = center;
    obs.radius = radius;

    m_obstacles.push_back(obs);
}

bool Planar2LinkCollisionChecker::isNoCollision(const Eigen::Vector2d& joint_angles) const
{
    // End effector position
    Eigen::Vector2d C { m_kinematics.forwardKinematics(joint_angles[0], joint_angles[1]) };

    // First elbow position
    Eigen::Vector2d B {
        m_kinematics.l1() * std::cos(joint_angles[0]),
        m_kinematics.l1() * std::sin(joint_angles[0])
    };

    Eigen::Vector2d A {0.0, 0.0};

    // We calculate now the distance between the segments (links) and the obstacles
    Eigen::Vector2d AB { B - A };
    Eigen::Vector2d AC { C - A };
    Eigen::Vector2d BC { C - B };

    Eigen::Vector2d AO {};
    Eigen::Vector2d BO {};
    Eigen::Vector2d closest {};

    for (const auto& obstacle : m_obstacles)
    {
        AO = obstacle.center - A;
        double t1 = AO.dot(AB) / AB.squaredNorm();
        double t1_clamp = std::clamp(t1, 0.0, 1.0);
        closest = A + t1_clamp * AB;
        if ((obstacle.center - closest).norm() < obstacle.radius)
            return false;

        BO = obstacle.center - B;
        double t2 = BO.dot(BC) / BC.squaredNorm();
        double t2_clamp = std::clamp(t2, 0.0, 1.0);
        closest = B + t2_clamp * BC;
        if ((obstacle.center - closest).norm() < obstacle.radius)
            return false;
    }
    return true;
}