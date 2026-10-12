cmake_minimum_required(VERSION 3.16)
project(coverage_planner_ros)

# ROS 2 service node around coverage_planner_core (~/compute_coverage_path).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(coverage_planner_core REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/GeoPolygon.msg
  srv/ComputeCoveragePath.srv
  DEPENDENCIES geometry_msgs mrs_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(coverage_planner_node SHARED src/coverage_planner_node.cpp)
target_link_libraries(coverage_planner_node
  rclcpp::rclcpp
  rclcpp_components::component
  ament_index_cpp::ament_index_cpp
  ${geometry_msgs_TARGETS}
  ${mrs_msgs_TARGETS}
  coverage_planner_core::coverage_planner_core
  "${cpp_typesupport_target}"
)
rclcpp_components_register_node(coverage_planner_node
  PLUGIN "coverage_planner_ros::CoveragePlannerNode"
  EXECUTABLE coverage_planner
)

install(TARGETS coverage_planner_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plan_client.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
