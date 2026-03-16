// MoveGroupInterface：MoveIt 2 的核心接口，用于规划和执行运动
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <openarm_moveit2_pose_executor/srv/execute_target_pose.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <mutex>
#include <optional>
#include <cmath>
#include <memory>

/*

使用说明：
屏蔽了四元数
ros2 launch openarm_moveit2_pose_executor motion_planner.launch.py group_name:=left_arm pose_topic_mode:=false position_only_mode:=true
没有屏蔽
ros2 launch openarm_moveit2_pose_executor motion_planner.launch.py group_name:=left_arm pose_topic_mode:=false position_only_mode:=false goal_orientation_tolerance:=0.15

*/

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planner_node");

// 将一组已知点（含途径点）作为笛卡尔路径进行规划并执行
static bool planAndExecute(moveit::planning_interface::MoveGroupInterface &move_group,
                                                     const std::vector<geometry_msgs::msg::Pose> &waypoints,
                                                     double eef_step = 0.005,
                                                     double min_fraction = 0.9,
                                                     bool fallback_to_pose_plan = true)
{
    if (waypoints.empty()) {
        RCLCPP_WARN(LOGGER, "planAndExecute：途径点列表为空");
        return false;
    }

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, traj);
    RCLCPP_INFO(LOGGER, "笛卡尔路径比例=%.3f", fraction);
    if (fraction >= min_fraction) {
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = traj;
        auto result = move_group.execute(cartesian_plan);
        const bool ok = static_cast<bool>(result);
        if (!ok) {
            RCLCPP_ERROR(LOGGER, "笛卡尔轨迹执行失败，MoveItErrorCode=%d", result.val);
        }
        return ok;
    }

    if (!fallback_to_pose_plan) {
        RCLCPP_WARN(LOGGER, "规划比例 %.3f < 最小比例 %.3f；中止", fraction, min_fraction);
        return false;
    }

    RCLCPP_WARN(LOGGER,
                "笛卡尔规划比例 %.3f < 最小比例 %.3f，回退到常规位姿规划",
                fraction,
                min_fraction);

    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(waypoints.back());

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    auto plan_result = move_group.plan(pose_plan);
    if (!static_cast<bool>(plan_result)) {
        RCLCPP_ERROR(LOGGER, "位姿规划失败，MoveItErrorCode=%d", plan_result.val);
        move_group.clearPoseTargets();
        return false;
    }

    auto exec_result = move_group.execute(pose_plan);
    move_group.clearPoseTargets();
    const bool ok = static_cast<bool>(exec_result);
    if (!ok) {
        RCLCPP_ERROR(LOGGER, "位姿轨迹执行失败，MoveItErrorCode=%d", exec_result.val);
    }
    return ok;
}

static bool planAndExecutePositionOnly(moveit::planning_interface::MoveGroupInterface &move_group,
                                       const geometry_msgs::msg::Pose &target_pose)
{
    move_group.setStartStateToCurrentState();
    move_group.setPositionTarget(target_pose.position.x,
                                 target_pose.position.y,
                                 target_pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group.plan(plan);
    if (!static_cast<bool>(plan_result)) {
        RCLCPP_ERROR(LOGGER, "位置规划失败，MoveItErrorCode=%d", plan_result.val);
        move_group.clearPoseTargets();
        return false;
    }

    auto exec_result = move_group.execute(plan);
    move_group.clearPoseTargets();
    const bool ok = static_cast<bool>(exec_result);
    if (!ok) {
        RCLCPP_ERROR(LOGGER, "位置轨迹执行失败，MoveItErrorCode=%d", exec_result.val);
    }
    return ok;
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

static void logTargetPose(const std::string &source,
                          const std::string &frame_id,
                          const geometry_msgs::msg::Pose &pose)
{
    const char *frame = frame_id.empty() ? "<empty>" : frame_id.c_str();
    RCLCPP_INFO(LOGGER,
                "%s target_pose: frame=%s, pos=[%.5f, %.5f, %.5f], quat=[%.6f, %.6f, %.6f, %.6f]",
                source.c_str(),
                frame,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w);
}

template <typename T>
static T getOrDeclareParameter(const rclcpp::Node::SharedPtr &node,
                               const std::string &name,
                               const T &default_value)
{
    T value = default_value;
    if (node->has_parameter(name)) {
        node->get_parameter(name, value);
    } else {
        value = node->declare_parameter<T>(name, default_value);
    }
    return value;
}

static bool sanitizeTargetPose(geometry_msgs::msg::Pose &pose,
                               const std::string &planning_group)
{
    const double qx = pose.orientation.x;
    const double qy = pose.orientation.y;
    const double qz = pose.orientation.z;
    const double qw = pose.orientation.w;
    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);

    if (norm < 1e-6) {
        RCLCPP_ERROR(LOGGER, "目标姿态四元数范数过小，无法规划");
        return false;
    }

    if (std::abs(norm - 1.0) > 1e-3) {
        pose.orientation.x = qx / norm;
        pose.orientation.y = qy / norm;
        pose.orientation.z = qz / norm;
        pose.orientation.w = qw / norm;
        RCLCPP_WARN(LOGGER, "目标四元数已自动归一化（原始范数=%.6f）", norm);
    }

    if (planning_group == "left_arm" && pose.position.y < 0.0) {
        RCLCPP_WARN(LOGGER, "当前为 left_arm，但目标 y=%.4f < 0，可能在右臂工作空间外", pose.position.y);
    }
    if (planning_group == "right_arm" && pose.position.y > 0.0) {
        RCLCPP_WARN(LOGGER, "当前为 right_arm，但目标 y=%.4f > 0，可能在左臂工作空间外", pose.position.y);
    }

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
    // 参数与规划组设置
    // ===============================
    // planning group 必须和 SRDF 中定义的名字完全一致
    const std::string planning_group =
        getOrDeclareParameter<std::string>(move_group_node, "group_name", "right_arm");
    const bool pose_topic_mode =
        getOrDeclareParameter<bool>(move_group_node, "pose_topic_mode", false);
    const std::string pose_topic =
        getOrDeclareParameter<std::string>(move_group_node, "pose_topic", "/target_pose");
    const bool pose_service_mode =
        getOrDeclareParameter<bool>(move_group_node, "pose_service_mode", true);
    const std::string pose_service_name =
        getOrDeclareParameter<std::string>(move_group_node, "pose_service_name", "/execute_target_pose");
    const double eef_step =
        getOrDeclareParameter<double>(move_group_node, "eef_step", 0.005);
    const double min_fraction =
        getOrDeclareParameter<double>(move_group_node, "min_fraction", 0.9);
    const bool fallback_to_pose_plan =
        getOrDeclareParameter<bool>(move_group_node, "fallback_to_pose_plan", true);
    const double planning_time =
        getOrDeclareParameter<double>(move_group_node, "planning_time", 10.0);
    const int num_planning_attempts =
        getOrDeclareParameter<int>(move_group_node, "num_planning_attempts", 10);
    const double goal_position_tolerance =
        getOrDeclareParameter<double>(move_group_node, "goal_position_tolerance", 0.01);
    const double goal_orientation_tolerance =
        getOrDeclareParameter<double>(move_group_node, "goal_orientation_tolerance", 0.05);
    const bool position_only_mode =
        getOrDeclareParameter<bool>(move_group_node, "position_only_mode", false);

    // 创建 MoveGroupInterface，用于该规划组的运动规划与执行
    moveit::planning_interface::MoveGroupInterface move_group(
        move_group_node, planning_group);
    move_group.setPlanningTime(planning_time);
    move_group.setNumPlanningAttempts(num_planning_attempts);
    move_group.setGoalPositionTolerance(goal_position_tolerance);
    move_group.setGoalOrientationTolerance(goal_orientation_tolerance);

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

    RCLCPP_INFO(LOGGER, "当前规划组：%s", planning_group.c_str());
    RCLCPP_INFO(LOGGER, "笛卡尔参数：eef_step=%.4f, min_fraction=%.3f, fallback_to_pose_plan=%s",
                eef_step,
                min_fraction,
                fallback_to_pose_plan ? "true" : "false");
    RCLCPP_INFO(LOGGER,
                "位姿规划参数：planning_time=%.2f, attempts=%d, pos_tol=%.4f, ori_tol=%.4f",
                planning_time,
                num_planning_attempts,
                goal_position_tolerance,
                goal_orientation_tolerance);
    RCLCPP_INFO(LOGGER, "位置仅模式：%s", position_only_mode ? "true（忽略姿态）" : "false");
    RCLCPP_INFO(LOGGER, "服务模式：%s, service=%s",
                pose_service_mode ? "true" : "false",
                pose_service_name.c_str());

    RCLCPP_INFO(LOGGER, "可用的规划组：");
    std::copy(move_group.getJointModelGroupNames().begin(),
              move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    std::mutex move_group_mutex;

    auto execute_target_pose = [&](geometry_msgs::msg::Pose target_pose,
                                   const std::string &target_frame,
                                   const std::string &request_source) -> std::pair<bool, std::string> {
        const std::string planning_frame = move_group.getPlanningFrame();
        const std::string effective_frame = target_frame.empty() ? planning_frame : target_frame;
        logTargetPose(request_source, effective_frame, target_pose);
        std::lock_guard<std::mutex> lock(move_group_mutex);
        move_group.setPoseReferenceFrame(effective_frame);
        if (!position_only_mode && !sanitizeTargetPose(target_pose, planning_group)) {
            return {false, "目标姿态非法：四元数无效或不符合当前规划组工作空间"};
        }
        const bool ok = position_only_mode
                            ? planAndExecutePositionOnly(move_group, target_pose)
                            : planAndExecute(move_group, std::vector<geometry_msgs::msg::Pose>{target_pose}, eef_step, min_fraction, fallback_to_pose_plan);
        return {
            ok,
            request_source + std::string(ok ? "执行成功" : "执行失败")
        };
    };

    rclcpp::Service<openarm_moveit2_pose_executor::srv::ExecuteTargetPose>::SharedPtr pose_service;
    if (pose_service_mode) {
        pose_service = move_group_node->create_service<openarm_moveit2_pose_executor::srv::ExecuteTargetPose>(
            pose_service_name,
            [&](const std::shared_ptr<openarm_moveit2_pose_executor::srv::ExecuteTargetPose::Request> request,
                std::shared_ptr<openarm_moveit2_pose_executor::srv::ExecuteTargetPose::Response> response) {
                logTargetPose("service 收到", request->target_pose.header.frame_id, request->target_pose.pose);
                auto result = execute_target_pose(
                    request->target_pose.pose,
                    request->target_pose.header.frame_id,
                    "service 请求：");
                response->success = result.first;
                response->message = result.second;
                RCLCPP_INFO(LOGGER, "收到 service 目标并执行：%s", response->success ? "成功" : "失败");
            });
        RCLCPP_INFO(LOGGER, "位姿执行服务已启动：%s", pose_service_name.c_str());
    }

    if (pose_topic_mode) {
        RCLCPP_INFO(LOGGER, "话题模式已启用，订阅目标位姿话题：%s", pose_topic.c_str());

        std::mutex target_pose_mutex;
        std::optional<geometry_msgs::msg::PoseStamped> pending_target_pose;

        auto pose_sub = move_group_node->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic,
            10,
            [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(target_pose_mutex);
                pending_target_pose = *msg;
                logTargetPose("topic 收到", msg->header.frame_id, msg->pose);
                RCLCPP_INFO(LOGGER, "收到目标位姿消息，等待执行...");
            });

        (void)pose_sub;
        while (rclcpp::ok()) {
            std::optional<geometry_msgs::msg::PoseStamped> next_pose;
            {
                std::lock_guard<std::mutex> lock(target_pose_mutex);
                if (pending_target_pose.has_value()) {
                    next_pose = pending_target_pose;
                    pending_target_pose.reset();
                }
            }

            if (next_pose.has_value()) {
                auto result = execute_target_pose(next_pose->pose, next_pose->header.frame_id, "topic 请求：");
                bool ok = result.first;
                RCLCPP_INFO(LOGGER, "收到话题目标并执行：%s", ok ? "成功" : "失败");
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
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
            if (!position_only_mode && !sanitizeTargetPose(target_pose, planning_group)) {
                RCLCPP_INFO(LOGGER, "本次规划执行：失败");
                continue;
            }

            logTargetPose("terminal 输入", move_group.getPlanningFrame(), target_pose);

            bool ok;
            {
                std::lock_guard<std::mutex> lock(move_group_mutex);
                ok = position_only_mode
                         ? planAndExecutePositionOnly(move_group, target_pose)
                         : planAndExecute(move_group, std::vector<geometry_msgs::msg::Pose>{target_pose}, eef_step, min_fraction, fallback_to_pose_plan);
            }
            RCLCPP_INFO(LOGGER, "本次规划执行：%s", ok ? "成功" : "失败");
        }
    }

    
    rclcpp::shutdown();
    return 0;
}



