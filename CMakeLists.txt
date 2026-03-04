cmake_minimum_required(VERSION 3.8)
project(openarm_moveit2_pose_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 以下行跳过检查版权的 linter
  # 当所有源文件都添加了版权和许可证后，请注释此行
  set(ament_cmake_copyright_FOUND TRUE)
  # 以下行跳过 cpplint（仅在 git 仓库中生效）
  # 当此包在 git 仓库中并且所有源文件都添加了版权和许可证后，请注释此行
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(openarm_moveit2_pose_executor_node src/motion_planner_node.cpp)
ament_target_dependencies(openarm_moveit2_pose_executor_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
)

add_executable(move_group_demo_openarm src/move_group_demo_openarm.cpp)
ament_target_dependencies(move_group_demo_openarm
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_visual_tools
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  move_group_demo_openarm
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  openarm_moveit2_pose_executor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config src
        DESTINATION share/${PROJECT_NAME})
ament_package()
