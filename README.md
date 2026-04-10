# openarm_moveit2_pose_executor

ROS 2 + MoveIt 2 位姿执行包。

当前主要功能：
- 启动 MoveGroupInterface 后，支持通过话题接收目标位姿（默认）。
- 启动位姿执行 service（默认 `/execute_target_pose`），支持请求-执行-返回结果。
- 提供基于 MoveIt Task Constructor（MTC）的抓取放置 service（默认 `/execute_pick_place`）。
- 支持可选的终端交互输入目标位姿。
- 对输入位姿执行笛卡尔路径规划并执行。
- 支持重复接收多组位姿，逐次规划执行。

## 包内容

- `src/motion_planner_node.cpp`：支持话题模式与交互模式的位姿规划执行节点。
- `srv/ExecuteTargetPose.srv`：位姿执行服务接口（请求 PoseStamped，响应 success/message）。
- `src/move_group_demo_openarm.cpp`：MoveIt 示例风格演示代码（含可视化/障碍物流程）。
- `src/mtc_pick_place_node.cpp`：基于 MTC 的抓取放置服务节点。
- `launch/motion_planner.launch.py`：启动主节点并加载 MoveIt 配置与路径参数。
- `launch/mtc_pick_place.launch.py`：启动 MTC 抓取放置节点。
- `config/path.yaml`：路径参数文件（按需使用）。

## 依赖

核心依赖（见 `package.xml`）：
- `rclcpp`
- `geometry_msgs`
- `moveit_ros_planning_interface`
- `moveit_msgs`
- `moveit_core`
- `moveit_visual_tools`
- `tf2_geometry_msgs`
- `tf2_ros`

## 使用方式

1. 启动你的 MoveIt/机器人环境（仿真或真机）。
2. 启动本包（默认话题模式，默认规划组 `left_arm`）：

```bash
ros2 launch openarm_moveit2_pose_executor motion_planner.launch.py
```

3. 发布目标位姿到话题（默认 `/target_pose`）。
	推荐先用下面这条 `left_arm` 的近距离测试点（已验证可动）：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.05, y: 0.18, z: 0.30}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

若使用 `right_arm`，可再尝试：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 0.15, y: -0.26, z: 0.49}, orientation: {x: 0.72512, y: 0.0, z: 0.68862, w: 0.0}}}"
```

4. 可选：通过 launch 参数切换规划组或话题名：

```bash
ros2 launch openarm_moveit2_pose_executor motion_planner.launch.py \
	group_name:=right_arm \
	pose_topic_mode:=true \
	pose_topic:=/target_pose
```

5. 通过 service 请求位姿执行（推荐给 client 程序调用）：

```bash
ros2 service call /execute_target_pose openarm_moveit2_pose_executor/srv/ExecuteTargetPose "{target_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.05, y: 0.18, z: 0.30}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}"
```

5. 可选：切回终端交互输入模式：

```bash
ros2 launch openarm_moveit2_pose_executor motion_planner.launch.py pose_topic_mode:=false
```

交互输入格式：

```text
x y z qx qy qz qw
```

示例：

```text
0.15 -0.26 0.49 0.72512 0.0 0.68862 0.0
```

退出命令：
- `q`
- `quit`
- `exit`

或直接运行节点：

```bash
ros2 run openarm_moveit2_pose_executor motion_planner_node
```

## 多点连续移动（独立节点）

节点：`waypoint_sequence_node`（不修改 `motion_planner_node`）。

运行命令（推荐，自动加载 MoveIt 的 robot_description 参数）：

```bash
ros2 launch openarm_moveit2_pose_executor waypoint_sequence.launch.py \
	group_name:=left_arm \
	orientation_is_wxyz:=true \
	target_poses_raw:="-0.10367,0.15178,0.18325,0.99639,-5.6906e-06,1.8881e-05,-0.084869|-0.19058,0.29804,0.23138,0.99638,3.1934e-05,3.1091e-05,-0.084963"
```

说明：直接 `ros2 run` 启动会缺少 `robot_description` / `robot_description_semantic`，从而无法构建机器人模型。

可选：继续追加多组位姿（每组 7 个数，组间用 `|` 分隔）：

```bash
ros2 run openarm_moveit2_pose_executor waypoint_sequence_node --ros-args \
	-p group_name:=left_arm \
	-p orientation_is_wxyz:=true \
	-p target_poses_raw:="x,y,z,qw,qx,qy,qz|x,y,z,qw,qx,qy,qz"
```

可选参数：
- `stop_on_failure`：某个点失败后是否立即停止（默认 `false`）
- `wait_seconds_between_targets`：相邻目标点之间等待时间（默认 `0.2` 秒）
- `planning_time`：单个点规划超时（默认 `3.0` 秒）
- `num_planning_attempts`：单个点规划尝试次数（默认 `2`）
- `max_velocity_scaling_factor`：最大速度缩放（默认 `0.8`）
- `max_acceleration_scaling_factor`：最大加速度缩放（默认 `0.8`）
- `try_cartesian_first`：先尝试笛卡尔路径（默认 `true`，手动拖拽连续点通常更快）
- `cartesian_eef_step`：笛卡尔插值步长（默认 `0.01`）
- `cartesian_min_fraction`：笛卡尔最小通过比例（默认 `0.9`）
- `goal_orientation_tolerance`：姿态容差（默认 `3.14159`，更适合“主要按 xyz 走点”）
- `orientation_is_wxyz`：四元数输入顺序是否按 `w,x,y,z`（默认 `true`）
- `target_poses_raw`：必填，不再提供内置默认点

工作空间建议：
- `left_arm` 一般对应 `y > 0` 区域
- `right_arm` 一般对应 `y < 0` 区域

## MTC 抓取放置

节点：`mtc_pick_place_node`，通过 `openarm_moveit2_pose_executor/srv/ExecutePickPlace` 服务触发一次完整抓取放置流程。

### 1) 依赖说明

该节点依赖 MoveIt Task Constructor：`moveit_task_constructor_core`。

示例安装（Ubuntu + ROS 2）：

```bash
sudo apt install ros-${ROS_DISTRO}-moveit-task-constructor-core
```

> 若系统未安装该依赖，本包会自动跳过 `mtc_pick_place_node` 的编译，不影响其它节点。

### 2) 启动（单臂）

右臂：

```bash
ros2 launch openarm_moveit2_pose_executor mtc_pick_place.launch.py \
	arm_group:=right_arm hand_group:=right_gripper eef_name:=right_ee \
	hand_frame:=openarm_right_gripper_tip ik_frame_link:=openarm_right_gripper_tip \
	service_name:=/execute_pick_place_right node_name:=mtc_pick_place_right_node
```

左臂：

```bash
ros2 launch openarm_moveit2_pose_executor mtc_pick_place.launch.py \
	arm_group:=left_arm hand_group:=left_gripper eef_name:=left_ee \
	hand_frame:=openarm_left_gripper_tip ik_frame_link:=openarm_left_gripper_tip \
	service_name:=/execute_pick_place_left node_name:=mtc_pick_place_left_node
```

### 3) 启动（双臂）

```bash
ros2 launch openarm_moveit2_pose_executor mtc_pick_place_dual.launch.py
```

默认会同时启动两个服务：
- 右臂：`/execute_pick_place_right`
- 左臂：`/execute_pick_place_left`

### 4) 调用服务

右臂调用示例：

```bash
ros2 service call /execute_pick_place_right openarm_moveit2_pose_executor/srv/ExecutePickPlace "{\
object_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.273, y: -0.152, z: 0.304}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, \
pick_pose:   {header: {frame_id: 'world'}, pose: {position: {x: 0.273, y: -0.156, z: 0.290}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, \
place_pose:  {header: {frame_id: 'world'}, pose: {position: {x: 0.227, y: -0.422, z: 0.430}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, \
next_pose:   {header: {frame_id: 'world'}, pose: {position: {x: 0.227, y: -0.422, z: 0.450}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}"
```

左臂调用示例（通常使用 `y > 0` 区域）：

```bash
ros2 service call /execute_pick_place_left openarm_moveit2_pose_executor/srv/ExecutePickPlace "{\
object_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.273, y: 0.152, z: 0.304}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, \
pick_pose:   {header: {frame_id: 'world'}, pose: {position: {x: 0.273, y: 0.156, z: 0.290}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, \
place_pose:  {header: {frame_id: 'world'}, pose: {position: {x: 0.227, y: 0.422, z: 0.430}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}, \
next_pose:   {header: {frame_id: 'world'}, pose: {position: {x: 0.227, y: 0.422, z: 0.450}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}}"
```

### 5) 常用参数

- `pick_pose`: `[x, y, z, qx, qy, qz, qw]`
- `place_pose`: `[x, y, z, qx, qy, qz, qw]`
- `object_pose`: 目标物在世界坐标中的初始位姿
- `object_size`: 目标物尺寸（默认 box）
- `table_pose` / `table_size`: 桌面碰撞体参数
- `gripper_open_pose` / `gripper_close_pose`: 手爪命名姿态
- `arm_home_pose`: 任务结束回归命名姿态

## 说明

- `group_name` 需与你的 SRDF 中分组名一致（如 `left_arm` 或 `right_arm`）。
- `pose_topic_mode:=true` 时通过话题驱动规划；`false` 时使用终端输入。
