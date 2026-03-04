/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // 我们启动一个 SingleThreadedExecutor 来运行当前状态监视器，以获取机器人的状态信息。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // 开始教程
  //
  // 设置
  // ^^^^^
  //
  // MoveIt 将一组关节称为 "planning group"（规划组），并在 JointModelGroup 对象中存储它们。
  // 在 MoveIt 中，"planning group" 与 "joint model group" 的说法可互换使用。
  static const std::string PLANNING_GROUP = "panda_arm";

  // MoveGroupInterface 类可以通过你想要控制和规划的规划组名称来简单配置。
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // 我们将使用 PlanningSceneInterface 类来在虚拟场景中添加或移除碰撞物体。
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 为了性能，通常使用原始指针来引用规划组。
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // 可视化
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control 是一个可供检查的工具，允许用户通过 RViz 中的按钮和快捷键逐步执行高级脚本 */
  visual_tools.loadRemoteControl();

  // RViz 提供多种类型的标记，在本示例中我们将使用文本、圆柱体和球体
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface 演示", rvt::WHITE, rvt::XLARGE);

  // 批量发布用于减少在大规模可视化时发送到 RViz 的消息数量
  visual_tools.trigger();

  // 获取基本信息
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 我们可以打印机器人的参考坐标系名称。
  RCLCPP_INFO(LOGGER, "规划参考坐标系: %s", move_group.getPlanningFrame().c_str());

  // 我们也可以打印该组的末端执行器链接名称。
  RCLCPP_INFO(LOGGER, "末端执行器链接: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "可用的规划组:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // 开始演示
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next' 开始演示");

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // 规划到位姿目标
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // 我们可以为该规划组生成一个到末端执行器期望位姿的运动规划。
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // 现在调用规划器来计算路径并可视化。
  // 注意：此处仅做规划，不会让 move_group 实际移动机器人。
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "可视化计划1（位姿目标） %s", success ? "" : "失败");

  // 可视化规划
  // ^^^^^^^^^^^^^^^^^
  // 我们也可以在 RViz 中使用标记将规划以轨迹线的形式可视化。
  RCLCPP_INFO(LOGGER, "以轨迹线可视化计划1");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "位姿目标", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next' 继续演示");

  // 移动到位姿目标
  // ^^^^^^^^^^^^^^^^^^^^
  //
  // 将机器人移动到位姿目标与上面的规划类似，但需要调用 ``move()`` 函数。
  // 注意：此前设置的位姿目标仍然有效，机器人会尝试移动到该目标。
  // 在本教程中我们不调用该函数，因为它会阻塞并且需要一个可用的控制器来报告执行结果。

  /* Uncomment below line when working with a real robot */
  /* move_group.move(); */

  // 规划到关节空间目标
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 现在我们设置一个关节空间目标并向其移动，这将替换之前设置的位姿目标。
  // 为此，首先创建一个指向当前机器人状态的指针。
  // RobotState 对象包含当前的位置/速度/加速度等数据。
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // 现在修改其中一个关节，规划到新的关节空间目标并可视化该规划。
  joint_group_positions[0] = -1.0;  // radians
  bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
  RCLCPP_WARN(LOGGER, "目标关节位置超出限制，已对目标进行裁剪并继续规划");
  }

  // 我们将允许的最大速度和加速度降低到最大值的 5%。
  // 默认值为 10% (0.1)。
  // 可在 moveit_config 的 joint_limits.yaml 中设置首选默认值，或在代码中显式设置以实现更高速度。
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "可视化计划2（关节空间目标） %s", success ? "" : "失败");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "关节空间目标", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next' 继续演示");

  // 带路径约束的规划
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 可以为机器人上的某个链路指定路径约束。下面为规划组指定一个姿态约束与目标位姿。
  // 首先定义路径约束。
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // 现在，将其设置为该规划组的路径约束。
  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // 强制在关节空间中进行规划
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 根据具体的规划问题，MoveIt 会在 ``joint space``（关节空间）和 ``cartesian space``（笛卡尔空间）之间选择表示方法。
  // 在 ompl_planning.yaml 中将参数 ``enforce_joint_model_state_space:true`` 设置为 true 可以强制始终使用关节空间。
  // 默认情况下，带有方向约束的规划请求会在笛卡尔空间进行采样，然后通过调用 IK 作为生成采样器。
  // 强制使用关节空间会采用拒绝采样来寻找有效请求，可能会显著增加规划时间。
  //
  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So we need to set the start
  // state to a new pose.
  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // 现在我们将从新创建的起始状态规划到先前设置的位姿目标。
  move_group.setPoseTarget(target_pose1);

  // 带约束的规划可能较慢，因为每个采样都要调用逆运动学求解器。
  // 将规划时间从默认的 5 秒增加，以确保规划器有足够时间找到解。
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "可视化计划3（约束） %s", success ? "" : "失败");

  // 在 RViz 中可视化规划：
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "约束目标", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next' 继续演示");

  // 完成路径约束的使用后，请务必清除它。
  move_group.clearPathConstraints();

  // 笛卡尔路径
  // ^^^^^^^^^^^^^^^
  // 你可以通过为末端执行器指定一组路径点直接规划笛卡尔路径。注意这里我们从上面设置的新起始状态开始。
  // 初始位姿（起始状态）无需加入路径点列表，但加入有助于可视化。
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::msg::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // 向下

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // 向右

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // 向上并向左

  // 我们希望笛卡尔路径以 1cm 的分辨率插值，因此将笛卡尔平移的最大步长设置为 0.01。
  const double eef_step = 0.01;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);
  RCLCPP_INFO(LOGGER, "可视化计划4（笛卡尔路径） (%.2f%% 完成)", fraction * 100.0);

  // 在 RViz 中可视化该规划
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "笛卡尔路径", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next' 继续演示");

  // 笛卡尔运动通常需要较慢速度，例如接近物体时。目前无法通过 maxVelocityScalingFactor 设置笛卡尔路径的速度，
  // 需要手动为轨迹设置时间（详见链接）。欢迎提交改进的 PR。
  //
  // 你可以像下面这样执行轨迹。
  /* move_group.execute(trajectory); */

  // 向环境中添加物体
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // 首先，我们在无障碍物的情况下规划到另一个简单目标。
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::Pose another_pose;
  another_pose.orientation.w = 0;
  another_pose.orientation.x = -1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "可视化计划5（无障碍物） %s", success ? "" : "失败");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "无障碍目标", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(another_pose, "goal");
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next' 继续演示");

  // The result may look like this:
  //
  // .. image:: ./move_group_interface_tutorial_clear_path.gif
  //    :alt: animation showing the arm moving relatively straight toward the goal
  //
  // Now, let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // 物体的 id 用于标识该物体。
  collision_object.id = "box1";

  // 定义一个要添加到场景中的箱体。
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // 定义该箱体在 frame_id 下的位姿。
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.48;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // 现在，将碰撞物体添加到场景中（可通过 vector 添加多个物体）
  RCLCPP_INFO(LOGGER, "向场景中添加物体");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // 在 RViz 中显示状态文本，并等待 MoveGroup 接收并处理碰撞物体消息
  visual_tools.publishText(text_pose, "添加物体", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next'，当碰撞物体出现在 RViz 后继续");

  // Now, when we plan a trajectory it will avoid the obstacle.
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "可视化计划6（绕障碍移动） %s", success ? "" : "失败");
  visual_tools.publishText(text_pose, "障碍物目标", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // The result may look like this:
  //
  // .. image:: ./move_group_interface_tutorial_avoid_path.gif
  //    :alt: animation showing the arm moving avoiding the new obstacle
  //
  // Attaching objects to the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // You can attach an object to the robot, so that it moves with the robot geometry.
  // This simulates picking up the object for the purpose of manipulating it.
  // The motion planning should avoid collisions between objects as well.
  moveit_msgs::msg::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::msg::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // 我们为该圆柱定义参考系/位姿，使其出现在夹爪中。
  object_to_attach.header.frame_id = move_group.getEndEffectorLink();
  geometry_msgs::msg::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = 0.2;

  // 首先，我们将该物体添加到世界中（不使用 vector 容器）。
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // 然后，我们将该物体“附着”到机器人上。MoveIt 会使用 frame_id 来确定附着到哪个机器人链接。
  // 我们还需要告诉 MoveIt 该物体可以与夹爪的指部链接发生碰撞（即允许与这些链接发生接触）。
  // 你也可以直接使用 applyAttachedCollisionObject 来把物体附着到机器人上。
  RCLCPP_INFO(LOGGER, "将物体附着到机器人");
  std::vector<std::string> touch_links;
  touch_links.push_back("panda_rightfinger");
  touch_links.push_back("panda_leftfinger");
  move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);

  visual_tools.publishText(text_pose, "物体已附着到机器人", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 等待 MoveGroup 接收并处理附着碰撞物体的消息 */
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next'，当新物体附着到机器人后继续");

  // Replan, but now with the object in hand.
  move_group.setStartStateToCurrentState();
  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "可视化计划7（夹持物后绕障移动） %s", success ? "" : "失败");
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next'，当规划完成后继续");

  // The result may look something like this:
  //
  // .. image:: ./move_group_interface_tutorial_attached_object.gif
  //    :alt: animation showing the arm moving differently once the object is attached
  //
  // Detaching and Removing Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Now, let's detach the cylinder from the robot's gripper.
  RCLCPP_INFO(LOGGER, "从机器人上分离物体");
  move_group.detachObject(object_to_attach.id);

  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "物体已从机器人分离", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 等待 MoveGroup 接收并处理分离物体的消息 */
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next'，当新物体从机器人上分离后继续");

  // Now, let's remove the objects from the world.
  RCLCPP_INFO(LOGGER, "从场景中移除物体");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "物体已从场景移除", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 等待 MoveGroup 接收并处理移除物体的消息 */
  visual_tools.prompt("在 RvizVisualToolsGui 窗口中按 'next'，当碰撞物体从场景移除后继续");

  // END_TUTORIAL
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}