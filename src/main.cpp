#include <iostream>
#include <iomanip>

#include "planar_2link_kinematics.hpp"
#include "joint_trayectory.hpp"

int main()
{
    // Set up the arm
    Planar2LinkKinematics arm{1.0, 1.0};

    // Define joint start and end positions (in radians)
    Eigen::Vector2d q_start{0.0, 0.0};
    Eigen::Vector2d q_end{M_PI / 2, M_PI / 4};  // 90° and 45°
    double duration = 2.0;  // seconds

    // Set up the trajectory
    JointTrayectory trajectory{q_start, q_end, duration};

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Time\tq1\tq2\tEE_x\tEE_y\n";

    // Simulate over time
    const double dt = 0.2;
    for (double t = 0.0; t <= duration; t += dt)
    {
        Eigen::Vector2d q = trajectory.getPosition(t);
        Eigen::Vector2d ee = arm.forwardKinematics(q[0], q[1]);

        std::cout << t << "\t" << q[0] << "\t" << q[1]
                  << "\t" << ee[0] << "\t" << ee[1] << "\n";
    }

    return 0;
}
