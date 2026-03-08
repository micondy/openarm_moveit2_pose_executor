#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("waypoint_sequence_node");

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

static std::vector<std::string> splitByDelimiter(const std::string &input, char delimiter)
{
  std::vector<std::string> parts;
  std::string current;
  for (char c : input) {
    if (c == delimiter) {
      if (!current.empty()) {
        parts.push_back(current);
        current.clear();
      }
      continue;
    }
    current.push_back(c);
  }
  if (!current.empty()) {
    parts.push_back(current);
  }
  return parts;
}

static bool parsePoseText(const std::string &text,
                          bool orientation_is_wxyz,
                          geometry_msgs::msg::Pose &pose)
{
  std::string cleaned = text;
  for (char &c : cleaned) {
    if (c == ',' || c == ';') {
      c = ' ';
    }
  }

  for (char &c : cleaned) {
    if (!std::isalnum(static_cast<unsigned char>(c)) &&
        c != '.' && c != '-' && c != '+' && c != 'e' && c != 'E' &&
        !std::isspace(static_cast<unsigned char>(c))) {
      c = ' ';
    }
  }

  std::istringstream iss(cleaned);
  double x, y, z, a, b, c, d;
  if (!(iss >> x >> y >> z >> a >> b >> c >> d)) {
    return false;
  }
  double extra;
  if (iss >> extra) {
    return false;
  }

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  if (orientation_is_wxyz) {
    pose.orientation.w = a;
    pose.orientation.x = b;
    pose.orientation.y = c;
    pose.orientation.z = d;
  } else {
    pose.orientation.x = a;
    pose.orientation.y = b;
    pose.orientation.z = c;
    pose.orientation.w = d;
  }

  const double qx = pose.orientation.x;
  const double qy = pose.orientation.y;
  const double qz = pose.orientation.z;
  const double qw = pose.orientation.w;
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm < 1e-9) {
    return false;
  }
  if (std::abs(norm - 1.0) > 1e-6) {
    pose.orientation.x = qx / norm;
    pose.orientation.y = qy / norm;
    pose.orientation.z = qz / norm;
    pose.orientation.w = qw / norm;
  }

  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("waypoint_sequence_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  const std::string planning_group =
      getOrDeclareParameter<std::string>(node, "group_name", "right_arm");
  const double planning_time = getOrDeclareParameter<double>(node, "planning_time", 3.0);
  const int num_planning_attempts = getOrDeclareParameter<int>(node, "num_planning_attempts", 2);
  const double goal_position_tolerance =
      getOrDeclareParameter<double>(node, "goal_position_tolerance", 0.01);
  const double goal_orientation_tolerance =
      getOrDeclareParameter<double>(node, "goal_orientation_tolerance", 3.14159);
  const double max_velocity_scaling_factor =
      getOrDeclareParameter<double>(node, "max_velocity_scaling_factor", 0.8);
  const double max_acceleration_scaling_factor =
      getOrDeclareParameter<double>(node, "max_acceleration_scaling_factor", 0.8);
  const bool try_cartesian_first =
      getOrDeclareParameter<bool>(node, "try_cartesian_first", true);
  const double cartesian_eef_step =
      getOrDeclareParameter<double>(node, "cartesian_eef_step", 0.01);
  const double cartesian_min_fraction =
      getOrDeclareParameter<double>(node, "cartesian_min_fraction", 0.9);
  const double wait_seconds_between_targets =
      getOrDeclareParameter<double>(node, "wait_seconds_between_targets", 0.2);
  const bool stop_on_failure = getOrDeclareParameter<bool>(node, "stop_on_failure", false);
  const bool orientation_is_wxyz =
      getOrDeclareParameter<bool>(node, "orientation_is_wxyz", true);
  const std::string target_poses_raw =
      getOrDeclareParameter<std::string>(node, "target_poses_raw", "");

  if (target_poses_raw.empty()) {
    RCLCPP_ERROR(LOGGER,
                 "参数 target_poses_raw 为空，节点退出。格式：x,y,z,qw,qx,qy,qz|x,y,z,qw,qx,qy,qz");
    rclcpp::shutdown();
    return 1;
  }

  const std::vector<std::string> raw_pose_entries = splitByDelimiter(target_poses_raw, '|');
  if (raw_pose_entries.empty()) {
    RCLCPP_ERROR(LOGGER, "参数 target_poses_raw 未解析出任何有效条目，节点退出。");
    rclcpp::shutdown();
    return 1;
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(raw_pose_entries.size());
  for (size_t index = 0; index < raw_pose_entries.size(); ++index) {
    geometry_msgs::msg::Pose pose;
    if (!parsePoseText(raw_pose_entries[index], orientation_is_wxyz, pose)) {
      RCLCPP_ERROR(LOGGER,
                   "target_poses_raw 第 %zu 项解析失败：%s（格式应为 x,y,z,qw,qx,qy,qz，使用 | 分隔）",
                   index,
                   raw_pose_entries[index].c_str());
      rclcpp::shutdown();
      return 1;
    }
    poses.push_back(pose);
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(num_planning_attempts);
  move_group.setGoalPositionTolerance(goal_position_tolerance);
  move_group.setGoalOrientationTolerance(goal_orientation_tolerance);
  move_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  move_group.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);

  const geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

  RCLCPP_INFO(LOGGER, "规划组: %s", planning_group.c_str());
  RCLCPP_INFO(LOGGER,
              "规划参数: planning_time=%.2f, attempts=%d, vel_scale=%.2f, acc_scale=%.2f, cartesian_first=%s",
              planning_time,
              num_planning_attempts,
              max_velocity_scaling_factor,
              max_acceleration_scaling_factor,
              try_cartesian_first ? "true" : "false");
  RCLCPP_INFO(LOGGER,
              "当前末端位姿: x=%.6f, y=%.6f, z=%.6f",
              current_pose.position.x,
              current_pose.position.y,
              current_pose.position.z);
  RCLCPP_INFO(LOGGER, "将依次执行 %zu 个目标位姿。", poses.size());

  size_t positive_y_count = 0;
  size_t negative_y_count = 0;
  for (const auto &pose : poses) {
    if (pose.position.y > 0.0) {
      ++positive_y_count;
    } else if (pose.position.y < 0.0) {
      ++negative_y_count;
    }
  }
  if (planning_group == "right_arm" && positive_y_count > negative_y_count) {
    RCLCPP_WARN(LOGGER,
                "当前为 right_arm，但大多数目标点 y>0，可能在左臂工作空间，建议改 group_name:=left_arm 或调整 y 为负值。");
  }
  if (planning_group == "left_arm" && negative_y_count > positive_y_count) {
    RCLCPP_WARN(LOGGER,
                "当前为 left_arm，但大多数目标点 y<0，可能在右臂工作空间，建议改 group_name:=right_arm 或调整 y 为正值。");
  }

  bool all_success = true;
  (void)current_pose;
  for (size_t index = 0; index < poses.size() && rclcpp::ok(); ++index) {
    const geometry_msgs::msg::Pose &target_pose = poses[index];

    RCLCPP_INFO(LOGGER,
                "[%zu/%zu] 目标位姿: x=%.6f, y=%.6f, z=%.6f, q=(%.6f, %.6f, %.6f, %.6f)",
                index + 1,
                poses.size(),
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);

    bool step_success = false;

    if (try_cartesian_first) {
      move_group.setStartStateToCurrentState();
      moveit_msgs::msg::RobotTrajectory cartesian_traj;
      const std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
      const double fraction =
          move_group.computeCartesianPath(waypoints, cartesian_eef_step, cartesian_traj);

      if (fraction >= cartesian_min_fraction) {
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory = cartesian_traj;
        auto cartesian_exec = move_group.execute(cartesian_plan);
        if (static_cast<bool>(cartesian_exec)) {
          step_success = true;
          RCLCPP_INFO(LOGGER,
                      "[%zu/%zu] 笛卡尔执行成功，fraction=%.3f",
                      index + 1,
                      poses.size(),
                      fraction);
        } else {
          RCLCPP_WARN(LOGGER,
                      "[%zu/%zu] 笛卡尔执行失败，MoveItErrorCode=%d，回退常规规划",
                      index + 1,
                      poses.size(),
                      cartesian_exec.val);
        }
      } else {
        RCLCPP_WARN(LOGGER,
                    "[%zu/%zu] 笛卡尔比例不足 fraction=%.3f(<%.3f)，回退常规规划",
                    index + 1,
                    poses.size(),
                    fraction,
                    cartesian_min_fraction);
      }
    }

    if (!step_success) {
      move_group.setStartStateToCurrentState();
      move_group.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto plan_result = move_group.plan(plan);
      if (!static_cast<bool>(plan_result)) {
        all_success = false;
        RCLCPP_ERROR(LOGGER,
                     "[%zu/%zu] 规划失败，MoveItErrorCode=%d",
                     index + 1,
                     poses.size(),
                     plan_result.val);
        move_group.clearPoseTargets();
        if (stop_on_failure) {
          break;
        }
        continue;
      }

      auto execute_result = move_group.execute(plan);
      move_group.clearPoseTargets();
      if (!static_cast<bool>(execute_result)) {
        all_success = false;
        RCLCPP_ERROR(LOGGER,
                     "[%zu/%zu] 执行失败，MoveItErrorCode=%d",
                     index + 1,
                     poses.size(),
                     execute_result.val);
        if (stop_on_failure) {
          break;
        }
      } else {
        step_success = true;
      }
    }

    if (step_success) {
      RCLCPP_INFO(LOGGER, "[%zu/%zu] 执行成功。", index + 1, poses.size());
    }

    if (index + 1 < poses.size() && wait_seconds_between_targets > 0.0) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(wait_seconds_between_targets));
    }
  }

  RCLCPP_INFO(LOGGER,
              "序列执行结束，结果：%s",
              all_success ? "全部成功" : "存在失败（请查看上方日志）");

  rclcpp::shutdown();
  return 0;
}
