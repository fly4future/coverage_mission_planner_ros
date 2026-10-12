#ifndef COVERAGE_PLANNER_CORE_PLANNER_HPP
#define COVERAGE_PLANNER_CORE_PLANNER_HPP

/*
 * ROS-free multi-UAV coverage planner.
 *
 * Wraps the EnergyAwareMCPP solver with the pipeline that used to live in iroc_fleet_manager's
 * CoveragePlanner plugin: zone validation, decomposition + MSTSP solve, assignment of the solved paths
 * to drones, transit paths from each drone's start and to its end slot, and transit-height
 * deconfliction. Input and output are latitude/longitude; projecting to a robot frame is the caller's job.
 */

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <EnergyAwareMCPP/SimpleLogger.h>
#include <coverage_planner_core/config.hpp>

namespace coverage_planner_core
{

struct GeoPoint
{
  double lat = 0.0;
  double lon = 0.0;
};

struct Zone
{
  std::vector<GeoPoint> polygon;  // open or closed, both are accepted
  double max_altitude = 0.0;      // no-fly zones only: 0 = no-fly at any altitude, >0 = ceiling of a height-restricted zone [m]
};

struct DroneInput
{
  std::string name;
  GeoPoint start;                        // where the drone is when it starts its coverage path
  double min_horizontal_distance = 5.0;  // separation to other drones' transit paths [m]
  double min_vertical_distance = 2.5;    // vertical separation used when transit paths overlap [m]
};

struct PlanRequest
{
  GeoPoint origin;  // origin of the local metric frame used internally
  std::vector<Zone> search_areas;
  std::vector<Zone> no_fly_zones;

  double sweep_height = 5.0;   // altitude of the sweep lanes [m]; transits fly above it
  double sweeping_step = 0.0;  // lane spacing [m]; <= 0 = AlgorithmParams default
  double max_altitude = 0.0;   // >0 = fail if any waypoint would be higher [m]

  std::vector<DroneInput> drones;
  // Where the drones must end. Empty = see return_to_start. Otherwise exactly one slot per drone, assigned to the
  // drones by sweep-end position (minimum total distance).
  std::vector<GeoPoint> end_slots;
  // Without end_slots: true = every drone flies back to its own start; false = each path ends where its sweep ends.
  bool return_to_start = true;

  DroneCostModel model;  // one model for every drone: the solver balances paths assuming identical drones
  AlgorithmParams params;
  std::optional<uint64_t> seed;  // unset = random; the seed actually used is reported in PlanResult
};

struct Waypoint
{
  double lat = 0.0;
  double lon = 0.0;
  double z = 0.0;  // same datum as PlanRequest::sweep_height
  double heading = 0.0;  // always 0 (as in the fleet manager plugin)
};

struct DronePlan
{
  std::string name;
  std::vector<Waypoint> waypoints;  // start transit + sweep + end transit
  size_t sweep_begin = 0;           // index of the first sweep waypoint (waypoints before it are transit)
  size_t sweep_end = 0;             // index one past the last sweep waypoint
  double cost = 0.0;                // in the model's unit (s for "time", J for "energy")
  bool has_sweep = false;           // false = the solver gave this drone no area; waypoints are a plain transit
  std::optional<size_t> end_slot;   // index into PlanRequest::end_slots
};

struct PlanResult
{
  bool success = false;
  std::string message;
  uint64_t seed = 0;
  std::vector<DronePlan> drones;  // one per requested drone, matched by name (never rely on the order)
};

PlanResult plan(const PlanRequest& request, std::shared_ptr<loggers::SimpleLogger> logger = nullptr);

}  // namespace coverage_planner_core

#endif  // COVERAGE_PLANNER_CORE_PLANNER_HPP
