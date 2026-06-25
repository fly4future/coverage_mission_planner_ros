#pragma once

#include "mrs_coverage_planner/utils/types.h"
#include <mrs_msgs/Reference.h>

#include <vector>

namespace mrs_coverage_planner {

// Helper functions to simplify the ROS API

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Point2D& point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  return msg_point;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Point3D& point) {
  Msg_T msg_point;
  msg_point.x = point.x;
  msg_point.y = point.y;
  msg_point.z = point.z;
  return msg_point;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Reference& point) {
  Msg_T msg_point;
  msg_point.position.x = point.x;
  msg_point.position.y = point.y;
  msg_point.position.z = point.z;
  msg_point.heading = point.heading;
  return msg_point;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Subtask& subtask) {
  Msg_T ros_subtask;
  ros_subtask.type = subtask.type;
  ros_subtask.parameters = subtask.parameters;
  ros_subtask.continue_without_waiting = subtask.continue_without_waiting;  
  ros_subtask.stop_on_failure = subtask.stop_on_failure;                   
  ros_subtask.max_retries = subtask.max_retries;                           
  ros_subtask.retry_delay = subtask.retry_delay;                           
  return ros_subtask;
}

template <typename Msg_T>
Msg_T toRosMsg(const custom_types::Waypoint& waypoint) {
  Msg_T ros_waypoint;

  ros_waypoint.reference = toRosMsg<mrs_msgs::Reference>(waypoint.reference);

    ros_waypoint.subtasks.reserve(waypoint.getSubtasks().size());
  for (const auto& subtask : waypoint.getSubtasks()) {
    // Assuming we define a local Subtask message or use a generic one. 
    // For now, I will keep the logic but we need to ensure the target type for toRosMsg exists.
    ros_waypoint.subtasks.emplace_back(toRosMsg<typename Msg_T::SubtaskType>(subtask));
  }

  ros_waypoint.parallel_execution = waypoint.parallel_execution;

  return ros_waypoint;
}

template <typename Msg_T, typename Point_T>
std::vector<Msg_T> toRosMsg(const std::vector<Point_T>& points) {
  std::vector<Msg_T> result;
  result.reserve(points.size());

  for (const auto& point : points) {
    result.emplace_back(toRosMsg<Msg_T>(point));
  }

  return result;
}
} // namespace mrs_coverage_planner
