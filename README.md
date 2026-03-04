# openarm_moveit2_pose_executor

ROS 2 + MoveIt 2 位姿执行包。

当前主要功能：
- 启动 MoveGroupInterface 后，支持通过话题接收目标位姿（默认）。
- 支持可选的终端交互输入目标位姿。
- 对输入位姿执行笛卡尔路径规划并执行。
- 支持重复接收多组位姿，逐次规划执行。

## 包内容

- `src/motion_planner_node.cpp`：支持话题模式与交互模式的位姿规划执行节点。
- `src/move_group_demo_openarm.cpp`：MoveIt 示例风格演示代码（含可视化/障碍物流程）。
- `launch/motion_planner.launch.py`：启动主节点并加载 MoveIt 配置与路径参数。
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

## 说明

- `group_name` 需与你的 SRDF 中分组名一致（如 `left_arm` 或 `right_arm`）。
- `pose_topic_mode:=true` 时通过话题驱动规划；`false` 时使用终端输入。
