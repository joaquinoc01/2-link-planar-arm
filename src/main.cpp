#include <iostream>
#include <iomanip>
#include <vector>

#include "joint_trayectory.hpp"
#include "planar_2link_kinematics.hpp"
#include "collision_checker.hpp"
#include "controller.hpp"

void runKinematicsTests()
{
    std::cout << "\n--- Kinematics Test ---\n";

    Planar2LinkKinematics kinematics(1.0, 1.0);

    double theta1 = M_PI / 4;
    double theta2 = M_PI / 4;
    Eigen::Vector2d ee = kinematics.forwardKinematics(theta1, theta2);
    std::cout << "Forward Kinematics:\nEnd Effector Position: [" << ee[0] << ", " << ee[1] << "]\n";

    auto ik_solutions = kinematics.inverseKinematics(ee[0], ee[1]);
    std::cout << "\nInverse Kinematics:\n";
    for (const auto& sol : ik_solutions)
        std::cout << "Theta1: " << sol[0] << ", Theta2: " << sol[1] << "\n";
}

void runTrajectoryTests()
{
    std::cout << "\n--- Trajectory Test ---\n";

    Eigen::Vector2d start{ 0.0, 0.0 };
    Eigen::Vector2d end{ M_PI / 2, M_PI / 2 };
    double duration = 2.0; // seconds

    JointTrayectory traj_linear(start, end, duration, InterpolationType::Linear);
    JointTrayectory traj_cubic(start, end, duration, InterpolationType::Cubic);

    std::cout << "Time\tLinear Pos\t\tLinear Vel\t\tCubic Pos\t\tCubic Vel\n";
    std::cout << std::fixed << std::setprecision(3);

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
}

void runCollisionCheckerTests()
{
    std::cout << "\n--- Collision Checker Tests ---\n";

    Planar2LinkKinematics kinematics(1.0, 1.0);
    Planar2LinkCollisionChecker collisionChecker(kinematics);

    // Add some obstacles
    collisionChecker.addObstacle(Eigen::Vector2d{0.5, 0.0}, 0.1);  // Near first link
    collisionChecker.addObstacle(Eigen::Vector2d{1.5, 0.0}, 0.1);  // Near second link
    collisionChecker.addObstacle(Eigen::Vector2d{2.0, 2.0}, 0.2);  // Far away, no collision

    // Test joint configurations
    std::vector<std::pair<Eigen::Vector2d, std::string>> testCases = {
        {{0.0, 0.0}, "Arm stretched out along x-axis"},
        {{M_PI / 4, M_PI / 4}, "Arm bent at 45 degrees"},
        {{M_PI / 2, 0.0}, "Arm pointing straight up first link"},
        {{M_PI / 2, M_PI / 2}, "Arm bent upward fully"},
    };

    for (const auto& [joint_angles, description] : testCases)
    {
        bool noCollision = collisionChecker.isNoCollision(joint_angles);
        std::cout << description << ": "
                  << (noCollision ? "No Collision" : "Collision Detected") << "\n";
    }
}

void runControlTests()
{
    std::cout << "\n--- Joint Space Controller Test ---\n";

    Eigen::Vector2d Kp{10.0, 10.0};
    Eigen::Vector2d Kd{2.0, 2.0};

    JointSpaceController controller(Kp, Kd);

    Eigen::Vector2d q_des{M_PI / 4, M_PI / 3};
    Eigen::Vector2d qd_des{0.0, 0.0};

    Eigen::Vector2d q{M_PI / 6, M_PI / 6};
    Eigen::Vector2d qd{0.1, -0.1};

    Eigen::Vector2d control = controller.computeControl(q_des, qd_des, q, qd);

    std::cout << "Control torques: [" << control[0] << ", " << control[1] << "]\n";
}

int main()
{
    runKinematicsTests();
    runTrajectoryTests();
    runCollisionCheckerTests();
    runControlTests();

    return 0;
}
