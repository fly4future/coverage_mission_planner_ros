#pragma once

#include <vector>
#include <string>
#include <utility>
#include <mrs_msgs/Reference.h>
#include <EnergyAwareMCPP/coverage_planner.hpp>

namespace mrs_coverage_planner {

using point_t = std::pair<double, double>;
using polygon_t = std::vector<point_t>;
using Waypoint = mrs_msgs::Reference;
using coverage_paths_t = std::vector<std::vector<Waypoint>>;

coverage_paths_t planStandaloneMission(
    const std::vector<point_t>& initial_drone_positions,
    const std::vector<polygon_t>& fly_zones,
    const std::vector<polygon_t>& no_fly_zones,
    const std::vector<std::pair<polygon_t, double>>& hr_no_fly_zones,
    std::vector<double> min_horizontal_distances,
    std::vector<double> min_vertical_distances,
    algorithm_config_t planner_config,
    double target_sweeping_height);

} // namespace mrs_coverage_planner
