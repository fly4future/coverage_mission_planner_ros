#include <mrs_coverage_planner/planner.h>
#include <ros/package.h>

#include <mrs_lib/param_loader.h>
#include <string>

// Energy aware coverage planner library includes
#include <EnergyAwareMCPP/EnergyCalculator.h>
#include <EnergyAwareMCPP/MapPolygon.hpp>
#include <EnergyAwareMCPP/ShortestPathCalculator.hpp>
#include <EnergyAwareMCPP/SimpleLogger.h>
#include <EnergyAwareMCPP/algorithms.hpp>
#include <EnergyAwareMCPP/coverage_planner.hpp>
#include <EnergyAwareMCPP/mstsp_solver/MstspSolver.h>
#include <EnergyAwareMCPP/mstsp_solver/SolverConfig.h>
#include <EnergyAwareMCPP/utils.hpp>

#include <mrs_coverage_planner/CoverageMission.h>
#include <mrs_coverage_planner/CoverageMissionRobot.h>
#include <mrs_coverage_planner/utils/conversions.h>
#include <mrs_msgs/Point2D.h>

namespace mrs_coverage_planner
{

namespace planners
{

namespace coverage_planner
{

class CoveragePlanner : public mrs_coverage_planner::planners::Planner {
public:
  bool initialize(const ros::NodeHandle &parent_nh, const std::string &name, const std::string &name_space) override;

  bool activate(void) override;
  void deactivate(void) override;
  std::tuple<result_t, std::vector<mrs_coverage_planner::CoverageMissionGoal>> createGoal(const std::string &goal) const override;

  std::string name_;

  // Additional type for coverage planner
  typedef std::vector<std::vector<mrs_coverage_planner::custom_types::Waypoint>> coverage_paths_t;

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  mutable algorithm_config_t planner_config_;

  algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const;

  coverage_paths_t getCoveragePaths(const mrs_coverage_planner::CoverageMission &mission, const std::vector<std::vector<custom_types::Point2DLatLon>> &search_areas, const std::vector<std::vector<custom_types::Point2DLatLon>> &no_fly_zones_arg, const std::vector<std::pair<std::vector<custom_types::Point2DLatLon>, double>> &hr_no_fly_zones_arg, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances) const;
};

} // namespace coverage_planner
} // namespace planners
} // namespace mrs_coverage_planner
