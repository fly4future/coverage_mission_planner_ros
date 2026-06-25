cmake_minimum_required(VERSION 3.0.2)
project(mrs_coverage_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Load standard ROS and MRS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrs_lib
  mrs_msgs
  geometry_msgs
  message_generation
)

# 2. Generate clean ROS messages and services
add_message_files(
  FILES
  CoverageMission.msg
  CoverageMissionRobot.msg
)

add_service_files(
  FILES
  ComputeCoveragePath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  mrs_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mrs_msgs message_runtime
)

include_directories(
  include
  lib/EnergyAwareMCPP/include
  ${catkin_INCLUDE_DIRS}
)

# 3. Compile the local EnergyAwareMCPP library bundle
add_library(energy_aware_mcpp
  lib/EnergyAwareMCPP/src/algorithms.cpp
  lib/EnergyAwareMCPP/src/EnergyCalculator.cpp
  lib/EnergyAwareMCPP/src/MapPolygon.cpp
  lib/EnergyAwareMCPP/src/ShortestPathCalculator.cpp
  lib/EnergyAwareMCPP/src/mstsp_solver/MstspSolver.cpp
  lib/EnergyAwareMCPP/src/mstsp_solver/TargetSet.cpp
  lib/EnergyAwareMCPP/src/utils.cpp
  lib/EnergyAwareMCPP/src/coverage_planner_lib.cpp
)

# 4. Compile your main ROS coordinator node execution binary
add_executable(mrs_coverage_planner_node 
  src/coverage_planner_node.cpp
  src/planner_plugins/coverage_planner.cpp
)

add_dependencies(mrs_coverage_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mrs_coverage_planner_node 
  ${catkin_LIBRARIES} 
  energy_aware_mcpp
)
