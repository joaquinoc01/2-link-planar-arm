#include <iostream>
#include <iomanip>

#include "joint_trayectory.hpp"
#include "planar_2link_kinematics.hpp"

int main()
{
    // Kinematics Test
    Planar2LinkKinematics kinematics(1.0, 1.0);

    double theta1 = M_PI / 4;
    double theta2 = M_PI / 4;
    Eigen::Vector2d ee = kinematics.forwardKinematics(theta1, theta2);
    std::cout << "Forward Kinematics:\nEnd Effector Position: [" << ee[0] << ", " << ee[1] << "]\n";

    auto ik_solutions = kinematics.inverseKinematics(ee[0], ee[1]);
    std::cout << "\nInverse Kinematics:\n";
    for (const auto& sol : ik_solutions)
        std::cout << "Theta1: " << sol[0] << ", Theta2: " << sol[1] << "\n";

    // Trajectory Test
    Eigen::Vector2d start{ 0.0, 0.0 };
    Eigen::Vector2d end{ M_PI / 2, M_PI / 2 };
    double duration = 2.0; // seconds

    JointTrayectory traj_linear(start, end, duration, InterpolationType::Linear);
    JointTrayectory traj_cubic(start, end, duration, InterpolationType::Cubic);

    std::cout << "\nTime\tLinear Pos\t\tLinear Vel\t\tCubic Pos\t\tCubic Vel\n";
    std::cout << std::fixed << std::setprecision(3);

    // Print vector with NO space immediately after '['
    auto printVec2 = [](const Eigen::Vector2d& v) {
        std::cout << "["
                  << std::setw(6) << std::right << v[0] << ", "
                  << std::setw(6) << std::right << v[1] << "]";
    };

    for (double t = 0.0; t <= duration; t += 0.2)
    {
        Eigen::Vector2d pos_lin = traj_linear.getPosition(t);
        Eigen::Vector2d vel_lin = traj_linear.getVelocity(t);

        Eigen::Vector2d pos_cub = traj_cubic.getPosition(t);
        Eigen::Vector2d vel_cub = traj_cubic.getVelocity(t);

        std::cout << std::setw(5) << t << "\t";
        printVec2(pos_lin);
        std::cout << "\t";
        printVec2(vel_lin);
        std::cout << "\t";
        printVec2(pos_cub);
        std::cout << "\t";
        printVec2(vel_cub);
        std::cout << "\n";
    }

    return 0;
}
