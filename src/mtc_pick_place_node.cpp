#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <openarm_moveit2_pose_executor/srv/execute_pick_place.hpp>

#include <memory>
#include <mutex>
#include <vector>
#include <cmath>
#include <string>
#include <chrono>
#include <sstream>
/*
# 终端 1：启动 MoveIt2 演示
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true

# 终端 2：启动简化版 pick-place 节点
# 右手
ros2 launch openarm_moveit2_pose_executor mtc_pick_place.launch.py \
  arm_group:=right_arm hand_group:=right_gripper eef_name:=right_ee \
  hand_frame:=openarm_right_gripper_tip ik_frame_link:=openarm_right_gripper_tip \
  service_name:=/execute_pick_place_right node_name:=mtc_pick_place_right_node

#左手
ros2 launch openarm_moveit2_pose_executor mtc_pick_place.launch.py \
  arm_group:=left_arm hand_group:=left_gripper eef_name:=left_ee \
  hand_frame:=openarm_left_gripper_tip ik_frame_link:=openarm_left_gripper_tip \
  service_name:=/execute_pick_place_left node_name:=mtc_pick_place_left_node

# 左右手 
ros2 launch openarm_moveit2_pose_executor mtc_pick_place_dual.launch.py

# 终端 3：调用服务
右
ros2 service call /execute_pick_place_right openarm_moveit2_pose_executor/srv/ExecutePickPlace "{object_pose:{header:{frame_id:'world'}}, pick_pose:{header:{frame_id:'world'}}, place_pose:{header:{frame_id:'world'}}, next_pose:{header:{frame_id:'world'}}}"
左
ros2 service call /execute_pick_place_left openarm_moveit2_pose_executor/srv/ExecutePickPlace "{object_pose:{header:{frame_id:'world'}}, pick_pose:{header:{frame_id:'world'}}, place_pose:{header:{frame_id:'world'}}, next_pose:{header:{frame_id:'world'}}}"

ros2 service call /execute_pick_place_right openarm_moveit2_pose_executor/srv/ExecutePickPlace "{
object_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.270, y: -0.200, z: 0.38}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, 
pick_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.270, y: -0.200, z: 0.38}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, 
place_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.17, y: -0.31, z: 0.366}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, 
next_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.17, y: -0.31, z: 0.386}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}"

*/

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_node");

template <typename T>
static T getOrDeclareParameter(const rclcpp::Node::SharedPtr &node,
                               const std::string &name,
                               const T &default_value)
{
  // 兼容“已声明参数”和“未声明参数”两种启动方式：若不存在则用默认值声明。
  T value = default_value;
  if (node->has_parameter(name)) {
    node->get_parameter(name, value);
  } else {
    value = node->declare_parameter<T>(name, default_value);
  }
  return value;
}

static geometry_msgs::msg::Pose toPose(const std::vector<double> &values)
{
  // 按 [x, y, z, qx, qy, qz, qw] 解析，长度不足时使用单位四元数。
  geometry_msgs::msg::Pose pose;
  if (values.size() >= 7) {
    pose.position.x = values[0];
    pose.position.y = values[1];
    pose.position.z = values[2];
    pose.orientation.x = values[3];
    pose.orientation.y = values[4];
    pose.orientation.z = values[5];
    pose.orientation.w = values[6];
  } else {
    pose.orientation.w = 1.0; 
  }
  return pose;
}

static std::vector<double> toVector(const geometry_msgs::msg::Pose &pose)
{
  return {pose.position.x, pose.position.y, pose.position.z,
          pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
}

class SimplePickPlaceNode
{
public:
  SimplePickPlaceNode()
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("mtc_pick_place_node", options);

    // 读取参数
    world_frame_ = getOrDeclareParameter<std::string>(node_, "world_frame", "world");
    arm_group_ = getOrDeclareParameter<std::string>(node_, "arm_group", "right_arm");
    hand_group_ = getOrDeclareParameter<std::string>(node_, "hand_group", "right_gripper");
    hand_frame_ = getOrDeclareParameter<std::string>(node_, "hand_frame", "openarm_right_gripper_tip");
    ik_frame_link_ = getOrDeclareParameter<std::string>(node_, "ik_frame_link", "openarm_right_gripper_tip");

    object_name_ = getOrDeclareParameter<std::string>(node_, "object_name", "target_object");
    table_name_ = getOrDeclareParameter<std::string>(node_, "table_name", "table");

    gripper_open_pose_ = getOrDeclareParameter<std::string>(node_, "gripper_open_pose", "open");
    gripper_close_pose_ = getOrDeclareParameter<std::string>(node_, "gripper_close_pose", "closed");

    pick_pose_vec_ = getOrDeclareParameter<std::vector<double>>(
      node_, "pick_pose", std::vector<double>{0.27323, -0.15564, 0.290, 1.0, 0.0, 0.0, 0.0});
    object_pose_vec_ = getOrDeclareParameter<std::vector<double>>(
      node_, "object_pose", std::vector<double>{0.27323, -0.15164, 0.30373, 0.0, 0.0, 0.0, 1.0});
    place_pose_vec_ = getOrDeclareParameter<std::vector<double>>(
      node_, "place_pose", std::vector<double>{0.27323, 0.15164, 0.290, 1.0, 0.0, 0.0, 0.0});
    next_pose_vec_ = getOrDeclareParameter<std::vector<double>>(
      node_, "next_pose", std::vector<double>{0.27323, 0.15164, 0.390, 1.0, 0.0, 0.0, 0.0});

    object_size_ = getOrDeclareParameter<std::vector<double>>(
        node_, "object_size", std::vector<double>{0.03, 0.03, 0.03});
    table_size_ = getOrDeclareParameter<std::vector<double>>(
        node_, "table_size", std::vector<double>{0.80, 1.20, 0.04});
    table_pose_vec_ = getOrDeclareParameter<std::vector<double>>(
        node_, "table_pose", std::vector<double>{0.35, 0.00, -0.02, 0.0, 0.0, 0.0, 1.0});

    approach_distance_ = getOrDeclareParameter<double>(node_, "approach_distance", 0.03);
    lift_distance_ = getOrDeclareParameter<double>(node_, "lift_distance", 0.12);
    line_down_eef_step_ = getOrDeclareParameter<double>(node_, "line_down_eef_step", 0.01);
    line_down_min_fraction_ = getOrDeclareParameter<double>(node_, "line_down_min_fraction", 0.70);
    allow_partial_line_down_ = getOrDeclareParameter<bool>(node_, "allow_partial_line_down", true);
    gripper_open_joint_value_ = getOrDeclareParameter<double>(node_, "gripper_open_joint_value", 0.044);
    gripper_close_joint_value_ = getOrDeclareParameter<double>(node_, "gripper_close_joint_value", 0.0);

    service_name_ = getOrDeclareParameter<std::string>(node_, "service_name", "/execute_pick_place");

    // 创建服务
    service_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = node_->create_service<openarm_moveit2_pose_executor::srv::ExecutePickPlace>(
        service_name_,
      std::bind(&SimplePickPlaceNode::handleService, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_callback_group_);

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "pick_place_visual_markers", rclcpp::QoS(1).transient_local());

    // 持久化 MoveGroup，避免在服务回调中临时创建导致当前状态监视器尚未就绪
    arm_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_);
    gripper_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, hand_group_);
    arm_move_group_->startStateMonitor();
    gripper_move_group_->startStateMonitor();

    RCLCPP_INFO(LOGGER, "简化版抓取服务已启动: %s", service_name_.c_str());
    RCLCPP_INFO(LOGGER, "参数: arm_group=%s, hand_group=%s, hand_frame=%s", 
                arm_group_.c_str(), hand_group_.c_str(), hand_frame_.c_str());
  }

  rclcpp::Node::SharedPtr getNode() const
  {
    return node_;
  }

private:
  std::string resolvePlanningTipLink() const
  {
    if (!ik_frame_link_.empty()) {
      return ik_frame_link_;
    }
    return hand_frame_;
  }

  std::string resolveGripperMainJointName() const
  {
    if (hand_group_.find("right") != std::string::npos) {
      return "openarm_right_finger_joint1";
    }
    return "openarm_left_finger_joint1";
  }

  static std::string joinStrings(const std::vector<std::string> &items)
  {
    std::ostringstream ss;
    for (size_t i = 0; i < items.size(); ++i) {
      if (i > 0) {
        ss << ", ";
      }
      ss << items[i];
    }
    return ss.str();
  }

  bool syncCurrentState(moveit::planning_interface::MoveGroupInterface &arm,
                        double timeout_sec,
                        const std::string &stage_name) const
  {
    (void)timeout_sec;
    arm.startStateMonitor();
    // 始终设置为当前状态。即使状态监视器不是最新，MoveIt2 也会使用最后已知的状态进行规划
    arm.setStartStateToCurrentState();
    RCLCPP_DEBUG(LOGGER, "[%s] 设置规划起始状态为当前状态", stage_name.c_str());
    return true;
  }

  void setupPlanningScene() const
  {
    geometry_msgs::msg::Pose object_pose = toPose(object_pose_vec_);
    geometry_msgs::msg::Pose table_pose = toPose(table_pose_vec_);

    // 仅显示，不参与碰撞检测：清理同名碰撞体并发布 RViz Marker
    planning_scene_interface_.removeCollisionObjects({table_name_, object_name_});

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker table_marker;
    table_marker.header.frame_id = world_frame_;
    table_marker.header.stamp = node_->now();
    table_marker.ns = "pick_place_visual";
    table_marker.id = 1;
    table_marker.type = visualization_msgs::msg::Marker::CUBE;
    table_marker.action = visualization_msgs::msg::Marker::ADD;
    table_marker.pose = table_pose;
    table_marker.scale.x = table_size_.size() > 0 ? table_size_[0] : 0.80;
    table_marker.scale.y = table_size_.size() > 1 ? table_size_[1] : 1.20;
    table_marker.scale.z = table_size_.size() > 2 ? table_size_[2] : 0.04;
    table_marker.color.r = 0.6f;
    table_marker.color.g = 0.6f;
    table_marker.color.b = 0.6f;
    table_marker.color.a = 0.35f;
    table_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(table_marker);

    visualization_msgs::msg::Marker object_marker;
    object_marker.header.frame_id = world_frame_;
    object_marker.header.stamp = node_->now();
    object_marker.ns = "pick_place_visual";
    object_marker.id = 2;
    object_marker.type = visualization_msgs::msg::Marker::CUBE;
    object_marker.action = visualization_msgs::msg::Marker::ADD;
    object_marker.pose = object_pose;
    object_marker.scale.x = object_size_.size() > 0 ? object_size_[0] : 0.03;
    object_marker.scale.y = object_size_.size() > 1 ? object_size_[1] : 0.03;
    object_marker.scale.z = object_size_.size() > 2 ? object_size_[2] : 0.10;
    object_marker.color.r = 0.2f;
    object_marker.color.g = 0.8f;
    object_marker.color.b = 0.3f;
    object_marker.color.a = 0.45f;
    object_marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(object_marker);

    marker_pub_->publish(marker_array);
    RCLCPP_INFO(LOGGER, "已发布可视化方块（无碰撞体积）: table=%s, object=%s",
                table_name_.c_str(), object_name_.c_str());
  }

  bool moveToPreGrasp(moveit::planning_interface::MoveGroupInterface &arm,
                      const std::string &planning_tip_link)
  {
    RCLCPP_INFO(LOGGER, "→ 移动到 pre-grasp（目标上方 3cm）");

    syncCurrentState(arm, 1.5, "pre_grasp");

    geometry_msgs::msg::Pose grasp_pose = toPose(pick_pose_vec_);
    geometry_msgs::msg::Pose pre_grasp_pose = grasp_pose;
    pre_grasp_pose.position.z += approach_distance_;  // 上方 3cm

    arm.setPoseTarget(pre_grasp_pose, planning_tip_link);
    arm.setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(LOGGER, "✓ Pre-grasp 规划成功，执行...");
      arm.execute(plan);
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "✗ Pre-grasp 规划失败");
      return false;
    }
  }

  bool lineDown(moveit::planning_interface::MoveGroupInterface &arm,
                const std::string &planning_tip_link)
  {
    RCLCPP_INFO(LOGGER, "→ 直线下降接近物体");

    const bool state_ok = syncCurrentState(arm, 1.5, "line_down");

    geometry_msgs::msg::Pose current_pose;
    if (state_ok) {
      current_pose = arm.getCurrentPose(planning_tip_link).pose;
    } else {
      // 无法可靠获取当前位姿时，使用几何先验（pre-grasp）作为下探起点
      current_pose = toPose(pick_pose_vec_);
      current_pose.position.z += approach_distance_;
    }
    geometry_msgs::msg::Pose grasp_pose = toPose(pick_pose_vec_);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);
    waypoints.push_back(grasp_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = arm.computeCartesianPath(waypoints, line_down_eef_step_, trajectory);

    const bool cartesian_ok = allow_partial_line_down_
                              ? (fraction >= line_down_min_fraction_)
                              : (fraction > 0.95);
    if (cartesian_ok) {
      if (fraction < 0.95) {
        RCLCPP_WARN(LOGGER,
                    "! Cartesian 路径部分可达 (%.1f%%，阈值 %.1f%%)，执行部分路径",
                    fraction * 100.0,
                    line_down_min_fraction_ * 100.0);
      } else {
        RCLCPP_INFO(LOGGER, "✓ Cartesian 路径规划成功 (%.1f%%)，执行...", fraction * 100.0);
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = trajectory;
      arm.execute(plan);
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "✗ Cartesian 路径规划不完整 (%.1f%%)，回退到位姿规划", fraction * 100.0);

      if (!state_ok) {
        RCLCPP_WARN(LOGGER, "✗ 当前状态不同步，跳过位姿回退规划");
        return false;
      }

      syncCurrentState(arm, 1.0, "line_down_fallback");
      arm.setPoseTarget(grasp_pose, planning_tip_link);
      arm.setPlanningTime(5.0);

      moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
      bool success = (arm.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      arm.clearPoseTargets();

      if (success) {
        RCLCPP_INFO(LOGGER, "✓ 位姿回退规划成功，执行...");
        arm.execute(pose_plan);
        return true;
      }

      RCLCPP_WARN(LOGGER, "✗ 位姿回退规划也失败");
      return false;
    }
  }

  bool closeGripper(moveit::planning_interface::MoveGroupInterface &gripper)
  {
    RCLCPP_INFO(LOGGER, "→ 闭合夹爪");

    syncCurrentState(gripper, 1.0, "gripper_close");

    constexpr int max_retries = 2;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      if (attempt > 1) {
        RCLCPP_WARN(LOGGER, "夹爪闭合重试 (%d/%d)...", attempt, max_retries);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        syncCurrentState(gripper, 1.0, "gripper_close_retry");
      }

      bool target_set = gripper.setNamedTarget(gripper_close_pose_);
      if (!target_set) {
        const auto named_targets = gripper.getNamedTargets();
        if (named_targets.empty()) {
          RCLCPP_WARN(LOGGER, "[第%d次] 无可用命名姿态，回退为关节值目标", attempt);
        } else {
          RCLCPP_WARN(LOGGER,
                      "[第%d次] 命名姿态 '%s' 不可用，可用: [%s]，用关节值目标",
                      attempt, gripper_close_pose_.c_str(),
                      joinStrings(named_targets).c_str());
        }
      }
      if (!target_set) {
        target_set = gripper.setJointValueTarget(resolveGripperMainJointName(), gripper_close_joint_value_);
      }
      if (!target_set) {
        RCLCPP_ERROR(LOGGER, "[第%d次] 无法设置目标", attempt);
        continue;
      }

      gripper.setPlanningTime(10.0);
      gripper.setMaxVelocityScalingFactor(0.5);
      gripper.setMaxAccelerationScalingFactor(0.5);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode result = gripper.plan(plan);
      gripper.clearPoseTargets();

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "✓ 夹爪闭合规划成功，执行...");
        gripper.execute(plan);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        return true;
      } else {
        RCLCPP_WARN(LOGGER, "[第%d次] 夹爪规划失败(错误: %d)", attempt, result.val);
      }
    }

    RCLCPP_ERROR(LOGGER, "✗ 夹爪闭合失败");
    return false;
  }

  bool openGripper(moveit::planning_interface::MoveGroupInterface &gripper)
  {
    RCLCPP_INFO(LOGGER, "→ 打开夹爪");

    syncCurrentState(gripper, 1.0, "gripper_open");

    // 若夹爪已接近打开值，直接视为成功，避免单关节小幅运动规划超时。
    const double open_tolerance = 0.003;
    const auto current_joint_values = gripper.getCurrentJointValues();
    if (!current_joint_values.empty() &&
        current_joint_values.front() >= (gripper_open_joint_value_ - open_tolerance)) {
      RCLCPP_INFO(LOGGER,
                  "夹爪当前已打开(%.4f >= %.4f)，跳过开爪规划",
                  current_joint_values.front(),
                  gripper_open_joint_value_ - open_tolerance);
      return true;
    }

    constexpr int max_retries = 2;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      if (attempt > 1) {
        RCLCPP_WARN(LOGGER, "夹爪打开重试 (%d/%d)...", attempt, max_retries);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        syncCurrentState(gripper, 1.0, "gripper_open_retry");
      }

      bool target_set = gripper.setNamedTarget(gripper_open_pose_);
      if (!target_set) {
        const auto named_targets = gripper.getNamedTargets();
        if (named_targets.empty()) {
          RCLCPP_WARN(LOGGER, "[第%d次] 无可用命名姿态，回退为关节值目标", attempt);
        } else {
          RCLCPP_WARN(LOGGER,
                      "[第%d次] 命名姿态 '%s' 不可用，可用: [%s]，用关节值目标",
                      attempt, gripper_open_pose_.c_str(),
                      joinStrings(named_targets).c_str());
        }
      }
      if (!target_set) {
        target_set = gripper.setJointValueTarget(resolveGripperMainJointName(), gripper_open_joint_value_);
      }
      if (!target_set) {
        RCLCPP_ERROR(LOGGER, "[第%d次] 无法设置目标", attempt);
        continue;
      }

      gripper.setPlanningTime(10.0);
      gripper.setMaxVelocityScalingFactor(0.5);
      gripper.setMaxAccelerationScalingFactor(0.5);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode result = gripper.plan(plan);
      gripper.clearPoseTargets();

      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, "✓ 夹爪打开规划成功，执行...");
        gripper.execute(plan);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        return true;
      } else {
        RCLCPP_WARN(LOGGER, "[第%d次] 夹爪规划失败(错误: %d)", attempt, result.val);
      }
    }

    // 最后兜底：虽然规划失败，但若状态已接近打开值，则继续流程。
    syncCurrentState(gripper, 1.0, "gripper_open_final_check");
    const auto final_joint_values = gripper.getCurrentJointValues();
    if (!final_joint_values.empty() &&
        final_joint_values.front() >= (gripper_open_joint_value_ - open_tolerance)) {
      RCLCPP_WARN(LOGGER,
                  "开爪规划失败，但夹爪已接近打开值(%.4f)，继续执行",
                  final_joint_values.front());
      return true;
    }

    RCLCPP_ERROR(LOGGER, "✗ 夹爪打开失败");
    return false;
  }

  bool liftUp(moveit::planning_interface::MoveGroupInterface &arm,
              const std::string &planning_tip_link)
  {
    RCLCPP_INFO(LOGGER, "→ 上升物体");

    syncCurrentState(arm, 1.5, "lift_up");

    // 避免依赖 getCurrentPose：直接从抓取位姿构造抬升目标
    geometry_msgs::msg::Pose lift_pose = toPose(pick_pose_vec_);
    lift_pose.position.z += lift_distance_;

    arm.setPoseTarget(lift_pose, planning_tip_link);
    arm.setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(LOGGER, "✓ 上升规划成功，执行...");
      arm.execute(plan);
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "✗ 上升规划失败");
      return false;
    }
  }

  bool moveToPlace(moveit::planning_interface::MoveGroupInterface &arm,
                   const std::string &planning_tip_link)
  {
    RCLCPP_INFO(LOGGER, "→ 移动到 place 位姿");

    syncCurrentState(arm, 1.5, "move_to_place");

    geometry_msgs::msg::Pose place_pose = toPose(place_pose_vec_);
    arm.setPoseTarget(place_pose, planning_tip_link);
    arm.setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    arm.clearPoseTargets();

    if (success) {
      RCLCPP_INFO(LOGGER, "✓ place 规划成功，执行...");
      arm.execute(plan);
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "✗ place 规划失败");
      return false;
    }
  }

  bool moveToNext(moveit::planning_interface::MoveGroupInterface &arm,
                  const std::string &planning_tip_link)
  {
    RCLCPP_INFO(LOGGER, "→ 移动到 next 位姿");

    syncCurrentState(arm, 1.5, "move_to_next");

    geometry_msgs::msg::Pose next_pose = toPose(next_pose_vec_);
    arm.setPoseTarget(next_pose, planning_tip_link);
    arm.setPlanningTime(5.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    arm.clearPoseTargets();

    if (success) {
      RCLCPP_INFO(LOGGER, "✓ next 规划成功，执行...");
      arm.execute(plan);
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "✗ next 规划失败");
      return false;
    }
  }

  void handleService(const std::shared_ptr<openarm_moveit2_pose_executor::srv::ExecutePickPlace::Request> request,
                     std::shared_ptr<openarm_moveit2_pose_executor::srv::ExecutePickPlace::Response> response)
  {
    std::lock_guard<std::mutex> lock(service_mutex_);

    RCLCPP_INFO(LOGGER, "\n========== 开始抓取流程 ==========");

    // 如果请求中提供了 pose，则使用之；否则使用从参数读取的默认值
    std::vector<double> effective_object_pose = object_pose_vec_;
    std::vector<double> effective_pick_pose = pick_pose_vec_;
    std::vector<double> effective_place_pose = place_pose_vec_;
    std::vector<double> effective_next_pose = next_pose_vec_;

    if (request->object_pose.header.frame_id != "") {
      effective_object_pose = toVector(request->object_pose.pose);
      RCLCPP_INFO(LOGGER, "使用请求提供的 object_pose: [%.3f, %.3f, %.3f, ...]",
                  effective_object_pose[0], effective_object_pose[1], effective_object_pose[2]);
    }
    if (request->pick_pose.header.frame_id != "") {
      effective_pick_pose = toVector(request->pick_pose.pose);
      RCLCPP_INFO(LOGGER, "使用请求提供的 pick_pose: [%.3f, %.3f, %.3f, ...]",
                  effective_pick_pose[0], effective_pick_pose[1], effective_pick_pose[2]);
    }
    if (request->place_pose.header.frame_id != "") {
      effective_place_pose = toVector(request->place_pose.pose);
      RCLCPP_INFO(LOGGER, "使用请求提供的 place_pose: [%.3f, %.3f, %.3f, ...]",
                  effective_place_pose[0], effective_place_pose[1], effective_place_pose[2]);
    }
    if (request->next_pose.header.frame_id != "") {
      effective_next_pose = toVector(request->next_pose.pose);
      RCLCPP_INFO(LOGGER, "使用请求提供的 next_pose: [%.3f, %.3f, %.3f, ...]",
                  effective_next_pose[0], effective_next_pose[1], effective_next_pose[2]);
    }

    // 临时替换成员变量以使用有效的位置
    auto old_object_pose = object_pose_vec_;
    auto old_pick_pose = pick_pose_vec_;
    auto old_place_pose = place_pose_vec_;
    auto old_next_pose = next_pose_vec_;
    object_pose_vec_ = effective_object_pose;
    pick_pose_vec_ = effective_pick_pose;
    place_pose_vec_ = effective_place_pose;
    next_pose_vec_ = effective_next_pose;

    const auto restorePoses = [&]() {
      object_pose_vec_ = old_object_pose;
      pick_pose_vec_ = old_pick_pose;
      place_pose_vec_ = old_place_pose;
      next_pose_vec_ = old_next_pose;
    };

    setupPlanningScene();

    auto &arm = *arm_move_group_;
    auto &gripper = *gripper_move_group_;

    const std::string planning_tip_link = resolvePlanningTipLink();
    if (planning_tip_link.empty()) {
      response->success = false;
      response->message = "tip link 为空，请设置 ik_frame_link 或 hand_frame";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      restorePoses();
      return;
    }

    // 显式指定末端执行器 link：确保 MoveIt2 使用 tip 进行 IK/位姿/笛卡尔规划
    if (!arm.setEndEffectorLink(planning_tip_link)) {
      response->success = false;
      response->message = std::string("设置 tip link 失败: ") + planning_tip_link;
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      restorePoses();
      return;
    }
    arm.setPoseReferenceFrame(world_frame_);
    RCLCPP_INFO(LOGGER, "规划末端链接(EEF link): %s", arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "位姿参考坐标系: %s", world_frame_.c_str());

    arm.setMaxVelocityScalingFactor(0.5);
    arm.setMaxAccelerationScalingFactor(0.5);

    syncCurrentState(arm, 3.0, "service_start");

    try {
      // 1. 移动到 pre-grasp
      if (!moveToPreGrasp(arm, planning_tip_link)) {
        response->success = false;
        response->message = "Pre-grasp 失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      // 2. 打开夹爪
      if (!openGripper(gripper)) {
        response->success = false;
        response->message = "夹爪打开失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }
      
      // 3. 直线下降
      if (!lineDown(arm, planning_tip_link)) {
        response->success = false;
        response->message = "直线下降失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      // 仅可视化模式下没有物体碰撞体，此处保留短暂停顿让状态稳定
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      // 4. 闭合夹爪
      if (!closeGripper(gripper)) {
        response->success = false;
        response->message = "夹爪闭合失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      // 5. 上升
      if (!liftUp(arm, planning_tip_link)) {
        response->success = false;
        response->message = "上升失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      // 6. 移动到 place
      if (!moveToPlace(arm, planning_tip_link)) {
        response->success = false;
        response->message = "移动到 place 失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      // 7. 放置：打开夹爪
      if (!openGripper(gripper)) {
        response->success = false;
        response->message = "放置开爪失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      // 8. 移动到 next
      if (!moveToNext(arm, planning_tip_link)) {
        response->success = false;
        response->message = "移动到 next 失败";
        RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
        restorePoses();
        return;
      }

      response->success = true;
      response->message = "抓取+放置+next 流程成功！";
      RCLCPP_INFO(LOGGER, "\n========== 抓取流程完成 ==========\n");
      restorePoses();

    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("异常: ") + e.what();
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      restorePoses();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<openarm_moveit2_pose_executor::srv::ExecutePickPlace>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  mutable std::mutex service_mutex_;
  mutable moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;

  std::string world_frame_;
  std::string arm_group_;
  std::string hand_group_;
  std::string hand_frame_;
  std::string ik_frame_link_;
  std::string object_name_;
  std::string table_name_;
  std::string gripper_open_pose_;
  std::string gripper_close_pose_;
  std::string service_name_;

  std::vector<double> pick_pose_vec_;
  std::vector<double> object_pose_vec_;
  std::vector<double> place_pose_vec_;
  std::vector<double> next_pose_vec_;
  std::vector<double> object_size_;
  std::vector<double> table_size_;
  std::vector<double> table_pose_vec_;

  double approach_distance_;
  double lift_distance_;
  double line_down_eef_step_;
  double line_down_min_fraction_;
  bool allow_partial_line_down_;
  double gripper_open_joint_value_;
  double gripper_close_joint_value_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto app = std::make_shared<SimplePickPlaceNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(app->getNode());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

