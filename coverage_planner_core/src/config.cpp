#include <coverage_planner_core/config.hpp>

#include <stdexcept>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace coverage_planner_core
{

namespace
{

YAML::Node require(const YAML::Node& node, const std::string& key, const std::string& where)
{
  if (!node[key]) {
    throw std::runtime_error("coverage planner config: missing key '" + key + "' in " + where);
  }
  return node[key];
}

}  // namespace

DroneCostModel load_cost_model(const YAML::Node& root)
{
  const YAML::Node drones = require(root, "drones", "the root");
  if (!drones.IsSequence() || drones.size() == 0) {
    throw std::runtime_error("coverage planner config: 'drones' must be a non-empty list");
  }
  // All drones share one model (the solver assumes identical drones); extra entries must agree on the type.
  const YAML::Node d = drones[0];
  DroneCostModel m;
  m.optimization_type = require(d, "optimization_type", "drones[0]").as<std::string>();
  m.max_single_path_cost = require(d, "max_single_path_cost", "drones[0]").as<double>();
  for (const auto& other : drones) {
    if (other["optimization_type"] && other["optimization_type"].as<std::string>() != m.optimization_type) {
      throw std::runtime_error("coverage planner config: all drones must have the same 'optimization_type'");
    }
  }

  if (m.optimization_type == "time") {
    m.time.max_speed = require(d, "max_speed", "drones[0]").as<double>();
    m.time.max_acceleration = require(d, "max_acceleration", "drones[0]").as<double>();
    m.time.allowed_path_deviation = require(d, "allowed_path_deviation", "drones[0]").as<double>();
  } else if (m.optimization_type == "energy") {
    auto& e = m.energy;
    e.drone_mass = require(d, "drone_mass", "drones[0]").as<double>();
    e.drone_area = require(d, "drone_area", "drones[0]").as<double>();
    e.average_acceleration = require(d, "average_acceleration", "drones[0]").as<double>();
    e.propeller_radius = require(d, "propeller_radius", "drones[0]").as<double>();
    e.number_of_propellers = require(d, "number_of_propellers", "drones[0]").as<int>();
    e.allowed_path_deviation = require(d, "allowed_path_deviation", "drones[0]").as<double>();
    const auto b = require(d, "battery_model", "drones[0]");
    e.battery_model = {require(b, "cell_capacity", "battery_model").as<double>(), require(b, "number_of_cells", "battery_model").as<int>(),
                       require(b, "d0", "battery_model").as<double>(),            require(b, "d1", "battery_model").as<double>(),
                       require(b, "d2", "battery_model").as<double>(),            require(b, "d3", "battery_model").as<double>()};
    const auto s = require(d, "best_speed_model", "drones[0]");
    e.best_speed_model = {require(s, "c0", "best_speed_model").as<double>(), require(s, "c1", "best_speed_model").as<double>(),
                          require(s, "c2", "best_speed_model").as<double>()};
    e.air_density = root["air_density"] ? root["air_density"].as<double>() : 1.225;
    e.earth_gravity = root["earth_gravity"] ? root["earth_gravity"].as<double>() : 9.81;
    e.propeller_efficiency = root["propeller_efficiency"] ? root["propeller_efficiency"].as<double>() : 0.6;
  } else {
    throw std::runtime_error("coverage planner config: unknown optimization_type '" + m.optimization_type + "' (use 'time' or 'energy')");
  }
  return m;
}

AlgorithmParams load_algorithm_params(const YAML::Node& root)
{
  AlgorithmParams p;
  if (root["number_of_rotations"]) p.number_of_rotations = root["number_of_rotations"].as<int>();
  if (root["decomposition_method"]) p.decomposition_type = static_cast<decomposition_type_t>(root["decomposition_method"].as<int>());
  if (root["min_sub_polygons_per_uav"]) p.min_sub_polygons_per_uav = root["min_sub_polygons_per_uav"].as<int>();
  if (root["rotations_per_cell"]) p.rotations_per_cell = root["rotations_per_cell"].as<int>();
  if (root["no_improvement_cycles_before_stop"]) p.no_improvement_cycles_before_stop = root["no_improvement_cycles_before_stop"].as<int>();
  if (root["sweeping_step"]) p.sweeping_step = root["sweeping_step"].as<double>();
  if (root["time_budget_s"]) p.time_budget_s = root["time_budget_s"].as<double>();
  return p;
}

void load_from_file(const std::string& path, DroneCostModel& model, AlgorithmParams& params, const std::string& sub_node)
{
  YAML::Node node = YAML::LoadFile(path);
  if (!sub_node.empty()) {
    std::stringstream ss(sub_node);
    std::string key;
    while (std::getline(ss, key, '.')) {
      if (!node[key]) {
        throw std::runtime_error("coverage planner config: '" + sub_node + "' not found in " + path);
      }
      const YAML::Node child = node[key];
      node.reset(child);  // rebind; operator= would overwrite the parent's content
    }
  }
  model = load_cost_model(node);
  params = load_algorithm_params(node);
}

}  // namespace coverage_planner_core
