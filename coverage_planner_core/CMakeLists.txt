cmake_minimum_required(VERSION 3.16)
project(coverage_planner_core)

# ROS-free multi-UAV coverage planner: the EnergyAwareMCPP solver (lib/). Only depends on yaml-cpp; ament_cmake is
# used for packaging.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

set(MCPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/EnergyAwareMCPP)

add_library(${PROJECT_NAME} SHARED
  ${MCPP_DIR}/src/coverage_planner.cpp
  ${MCPP_DIR}/src/MapPolygon.cpp
  ${MCPP_DIR}/src/TimeCalculator.cpp
  ${MCPP_DIR}/src/EnergyCalculator.cpp
  ${MCPP_DIR}/src/utils.cpp
  ${MCPP_DIR}/src/algorithms.cpp
  ${MCPP_DIR}/src/ShortestPathCalculator.cpp
  ${MCPP_DIR}/src/mstsp_solver/TargetSet.cpp
  ${MCPP_DIR}/src/mstsp_solver/MstspSolver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${MCPP_DIR}/include>
  $<BUILD_INTERFACE:${MCPP_DIR}/include/EnergyAwareMCPP>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/EnergyAwareMCPP>
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_TARGET})

# Command-line front end (the library's former main()): plans from a sample config and writes path_<i>.csv
add_executable(coverage_planner_cli tools/cli_main.cpp)
target_link_libraries(coverage_planner_cli ${PROJECT_NAME})

install(DIRECTORY ${MCPP_DIR}/include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS coverage_planner_cli RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${MCPP_DIR}/sample_configs ${MCPP_DIR}/scripts DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)
ament_package()
