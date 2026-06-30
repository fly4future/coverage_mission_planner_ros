#pragma once

#include <mrs_coverage_planner/utils/json_var_parser.h>
#include <ros/ros.h>

namespace mrs_coverage_planner {

namespace planners {

struct result_t {
  bool success;
  std::string message;
};

using json = nlohmann::json;

class Planner {
 public:
  /**
   * @brief Initializes the planner. It is called once for every planner. The
   * runtime is not limited.
   *
   * @param nh the node handle of the FleetManager
   * @param name of the planner for distinguishing multiple running instances of
   * the same code
   * @param name_space the parameter namespace of the planner, can be used
   * during initialization of the private node handle
   *
   * @return true if success
   */
  virtual bool initialize(const ros::NodeHandle& nh, const std::string& name, const std::string& name_space) = 0;

  /**
   * @brief It is called before the planner will be required and used. Should
   * not take much time (within miliseconds).
   *
   * @return true if success
   */
  virtual bool activate(void) = 0;

  /**
   * @brief is called when this planner is no longer needed. However, it can be
   * activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief Request for planner to process an incoming goal
   * @param incoming goal for the planner, string with JSON format type
   * @return the goals of the robots in the fleet.
   */
  virtual std::tuple<result_t, std::vector<mrs_coverage_planner::CoverageMissionGoal>> createGoal(const std::string& goal) const = 0;

  virtual ~Planner() = default;

 protected:
  result_t parseJson(const std::string& goal, json& json_msg) const;
};

result_t Planner::parseJson(const std::string& goal, json& json_msg) const {

  result_t result;
  try {
    json_msg = json::parse(goal);
  } catch (const json::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Planner]: Bad json input: " << e.what());
    result.success = false;
    result.message = "BadRequest_400: Bad JSON input";
    return result;
  }

  result.success = true;
  return result;
}

} // namespace planners 
} // namespace mrs_coverage_planner

