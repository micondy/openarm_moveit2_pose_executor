# openarm_moveit2_pose_executor

ROS 2 + MoveIt 2 位姿执行包。

当前主要功能：
- 启动 MoveGroupInterface 后，循环接收终端输入的目标位姿。
- 对输入位姿执行笛卡尔路径规划并执行。
- 支持重复输入多组位姿，逐次规划执行。

## 包内容

- `src/motion_planner_node.cpp`：交互式位姿输入与规划执行节点。
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
2. 启动本包：

```bash
ros2 launch openarm_moveit2_pose_executor motion_planner.launch.py
```

3. 在节点终端中按提示输入目标位姿：

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

## 说明

- 规划组当前使用 `right_arm`，需与你的 SRDF 中分组名一致。
- 后续可扩展为接收视觉模块输出位姿（替换或新增输入源即可）。
