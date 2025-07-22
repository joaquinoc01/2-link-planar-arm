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

    // We calculate now the distance between the segments (links) and the obstacles
    Eigen::Vector2d AB { B }; // x0, y0 is 0, 0
    Eigen::Vector2d AC { C };
    
    // Project AC onto AB
    double t = (AC.dot(AB)) / AB.squaredNorm();

    // Clamp t to [0, 1]
    double t_clamp = std::max(0.0, std::min(1.0, t));

    // Find the closest point on the segment
    double P = t_clamp * AB;

    // Euclidean distance from C to P
    double d = (C - P).norm();

    for (const auto& obstacle : m_obstacles)
    {

    }

}