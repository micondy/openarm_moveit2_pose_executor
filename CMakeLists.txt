cmake_minimum_required(VERSION 3.8)
project(openarm_moveit2_pose_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteTargetPose.srv"
  "srv/ExecutePickPlace.srv"
  DEPENDENCIES geometry_msgs
)


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

add_executable(motion_planner_node src/motion_planner_node.cpp)
ament_target_dependencies(motion_planner_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motion_planner_node "${cpp_typesupport_target}")

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

add_executable(waypoint_sequence_node src/waypoint_sequence_node.cpp)
ament_target_dependencies(waypoint_sequence_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
)

option(BUILD_MTC_PICK_PLACE "Build MTC pick-place node" ON)
if(BUILD_MTC_PICK_PLACE)
  find_package(moveit_task_constructor_core QUIET)
  if(moveit_task_constructor_core_FOUND)
    add_executable(mtc_pick_place_node src/mtc_pick_place_node.cpp)
    ament_target_dependencies(mtc_pick_place_node
      rclcpp
      std_srvs
      geometry_msgs
      shape_msgs
      visualization_msgs
      moveit_ros_planning_interface
      moveit_msgs
      moveit_core
      moveit_visual_tools
      moveit_task_constructor_core
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(mtc_pick_place_node "${cpp_typesupport_target}")
  else()
    message(WARNING "moveit_task_constructor_core not found, skip building mtc_pick_place_node")
  endif()
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS
  move_group_demo_openarm
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  motion_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  waypoint_sequence_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(TARGET mtc_pick_place_node)
  install(TARGETS
    mtc_pick_place_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch config src
        DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
