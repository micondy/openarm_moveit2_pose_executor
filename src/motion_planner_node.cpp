// MoveGroupInterface：MoveIt 2 的核心接口，用于规划和执行运动
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planner_node");

// 将一组已知点（含途径点）作为笛卡尔路径进行规划并执行
static bool planAndExecute(moveit::planning_interface::MoveGroupInterface &move_group,
                                                     const std::vector<geometry_msgs::msg::Pose> &waypoints,
                                                     double eef_step = 0.005,
                                                     double min_fraction = 0.9)
{
    if (waypoints.empty()) {
        RCLCPP_WARN(LOGGER, "planAndExecute：途径点列表为空");
        return false;
    }

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, traj);
    RCLCPP_INFO(LOGGER, "笛卡尔路径比例=%.3f", fraction);
    if (fraction < min_fraction) {
        RCLCPP_WARN(LOGGER, "规划比例 %.3f < 最小比例 %.3f；中止", fraction, min_fraction);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = traj;
    move_group.execute(plan);
    return true;
}

// 解析一行输入：x y z qx qy qz qw
static bool parsePoseLine(const std::string &line, geometry_msgs::msg::Pose &pose)
{
    std::istringstream iss(line);
    double x, y, z, qx, qy, qz, qw;
    if (!(iss >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        return false;
    }
    double extra;
    if (iss >> extra) {
        return false;
    }

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return true;
}

int main(int argc, char **argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // NodeOptions：允许从 launch / 参数服务器中自动声明参数（Jazzy 必需）
    // 在launch启动文件里面，moveit_config = MoveItConfigsBuilder("openarm", package_name="openarm_bimanual_moveit_config").to_moveit_configs()
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 创建一个 ROS 2 节点，用于 MoveIt 接口
    auto move_group_node =
        rclcpp::Node::make_shared("openarm_move_group", node_options);

    // 为 CurrentStateMonitor 单独启动一个 executor（用于实时获取机器人状态）
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    // ===============================
    // 规划组（Planning Group）设置
    // ===============================
    // planning group 必须和 SRDF 中定义的名字完全一致
    static const std::string PLANNING_GROUP = "right_arm";

    // 创建 MoveGroupInterface，用于该规划组的运动规划与执行
    moveit::planning_interface::MoveGroupInterface move_group(
        move_group_node, PLANNING_GROUP);

    // 用于管理规划场景中的障碍物
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 获取当前末端执行器位姿
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    RCLCPP_INFO(LOGGER, "当前末端执行器位姿：");
    RCLCPP_INFO(LOGGER, "位置 -> x: %.4f, y: %.4f, z: %.4f",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);
    RCLCPP_INFO(LOGGER, "朝向 -> x: %.6f, y: %.6f, z: %.6f, w: %.6f",
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w);

    // ===============================
    // 打印基础信息
    // ===============================
    RCLCPP_INFO(LOGGER, "规划参考坐标系：%s",
                move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(LOGGER, "末端执行器连杆：%s",
                move_group.getEndEffectorLink().c_str());

    RCLCPP_INFO(LOGGER, "可用的规划组：");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    RCLCPP_INFO(LOGGER,
                "请输入目标位姿：x y z qx qy qz qw；输入 q 或 quit 退出。\n"
                "示例: 0.15 -0.26 0.49 0.72512 0.0 0.68862 0.0");

    std::string line;
    while (rclcpp::ok()) {
        std::cout << "\n目标位姿> " << std::flush;
        if (!std::getline(std::cin, line)) {
            RCLCPP_INFO(LOGGER, "检测到输入结束，退出节点。");
            break;
        }

        if (line == "q" || line == "quit" || line == "exit") {
            RCLCPP_INFO(LOGGER, "收到退出指令，结束。\n");
            break;
        }

        if (line.empty()) {
            continue;
        }

        geometry_msgs::msg::Pose target_pose;
        if (!parsePoseLine(line, target_pose)) {
            RCLCPP_WARN(LOGGER, "输入格式错误，请按: x y z qx qy qz qw");
            continue;
        }

        std::vector<geometry_msgs::msg::Pose> path;
        path.push_back(target_pose);

        bool ok = planAndExecute(move_group, path);
        RCLCPP_INFO(LOGGER, "本次规划执行：%s", ok ? "成功" : "失败");
    }

    
    rclcpp::shutdown();
    return 0;
//ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true robot_controller:=forward_position_controller
//ros2 launch moveit_control start.launch.py
// ros2 pkg create openarm_moveit2_pose_executor \
//   --build-type ament_cmake \
//   --dependencies \
//     rclcpp \
//     geometry_msgs \
//     moveit_ros_planning_interface \
//     moveit_msgs \
//     moveit_core
}



