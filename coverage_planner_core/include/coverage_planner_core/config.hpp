#ifndef COVERAGE_PLANNER_CORE_CONFIG_HPP
#define COVERAGE_PLANNER_CORE_CONFIG_HPP

#include <string>

#include <EnergyAwareMCPP/EnergyCalculator.h>
#include <EnergyAwareMCPP/TimeCalculator.hpp>
#include <EnergyAwareMCPP/algorithms.hpp>

namespace YAML
{
class Node;
}

namespace coverage_planner_core
{

/// Cost model of a drone, used both to balance the paths and to check them against max_single_path_cost.
struct DroneCostModel
{
  std::string optimization_type = "time";  // "time" or "energy"
  time_calculator_config_t time{4.0, 2.0, 2.0};
  energy_calculator_config_t energy{};
  double max_single_path_cost = 1e9;  // [s] for "time", [J] for "energy"
};

/// Solver tuning, see EnergyAwareMCPP's sample configs for the meaning of each value.
struct AlgorithmParams
{
  int number_of_rotations = 3;
  decomposition_type_t decomposition_type = BOUSTROPHEDON_DECOMPOSITION;
  int min_sub_polygons_per_uav = 1;
  int rotations_per_cell = 4;
  int no_improvement_cycles_before_stop = 400;
  double sweeping_step = 6.0;   // default lane spacing [m]
  double time_budget_s = 0.0;   // wall-clock budget of the whole solve; 0 = unlimited
};

/*!
 * Read the cost model from a YAML node. Accepts the iroc_fleet_manager layout: the first entry of a `drones:` list
 * (all drones share one model), with `air_density`, `earth_gravity` and `propeller_efficiency` read from @p root.
 * Throws std::runtime_error with a readable message on missing keys.
 */
DroneCostModel load_cost_model(const YAML::Node& root);

/// Read AlgorithmParams from a YAML node; missing keys keep their defaults.
AlgorithmParams load_algorithm_params(const YAML::Node& root);

/// Convenience: load both from a YAML file (optionally from a sub-node path like "fleet_manager.planners.coverage_planner").
void load_from_file(const std::string& path, DroneCostModel& model, AlgorithmParams& params, const std::string& sub_node = "");

}  // namespace coverage_planner_core

#endif  // COVERAGE_PLANNER_CORE_CONFIG_HPP
