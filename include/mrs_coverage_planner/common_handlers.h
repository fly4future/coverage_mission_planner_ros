#pragma once

#include <memory.h>
#include <mrs_lib/mutex.h>
#include <mrs_robot_diagnostics/CollisionAvoidanceInfo.h>
#include <mrs_robot_diagnostics/ControlInfo.h>
#include <mrs_robot_diagnostics/GeneralRobotInfo.h>
#include <mrs_robot_diagnostics/StateEstimationInfo.h>
#include <mrs_robot_diagnostics/SystemHealthInfo.h>
#include <mrs_robot_diagnostics/UavInfo.h>
#include <mrs_robot_diagnostics/enums/robot_type.h>
#include <mrs_msgs/SafetyAreaManagerDiagnostics.h>
#include <string>
#include <unordered_map>

namespace mrs_coverage_planner {

// Note: ConstPtr type is a typedef for a shared pointer,
// specifically boost::shared_ptr in ROS 1

struct CommonRobotHandler_t {
  mrs_robot_diagnostics::GeneralRobotInfo::ConstPtr general_robot_info;
  mrs_robot_diagnostics::StateEstimationInfo::ConstPtr state_estimation_info;
  mrs_robot_diagnostics::ControlInfo::ConstPtr control_info;
  mrs_robot_diagnostics::CollisionAvoidanceInfo::ConstPtr collision_avoidance_info;
  mrs_robot_diagnostics::UavInfo::ConstPtr uav_info;
  mrs_robot_diagnostics::SystemHealthInfo::ConstPtr system_health_info;
  mrs_msgs::SafetyAreaManagerDiagnostics::ConstPtr safety_area_info;
};

struct CommonRobotHandlers_t {
  std::unordered_map<std::string, CommonRobotHandler_t> robots_map;
};

struct CommonHandlers_t {
  std::shared_ptr<CommonRobotHandlers_t> handlers;
};

} // namespace mrs_coverage_planner

