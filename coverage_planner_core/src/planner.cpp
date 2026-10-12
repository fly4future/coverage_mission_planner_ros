#include <coverage_planner_core/planner.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>

#include <EnergyAwareMCPP/coverage_planner.hpp>

#include "transit_heights.hpp"

namespace coverage_planner_core
{

namespace
{

using detail::path_t;

polygon_t to_polygon(const std::vector<GeoPoint>& points)
{
  polygon_t poly;
  poly.reserve(points.size() + 1);
  for (const auto& p : points) {
    poly.emplace_back(p.lat, p.lon);
  }
  if (!poly.empty() && poly.front() != poly.back()) {
    poly.push_back(poly.front());  // the library expects closed polygons
  }
  return poly;
}

// Ray casting on a closed polygon
bool is_inside(const point_t& p, const polygon_t& polygon)
{
  if (polygon.empty()) {
    return false;
  }
  int intersections = 0;
  for (size_t i = 0; i + 1 < polygon.size(); ++i) {
    const auto& p1 = polygon[i];
    const auto& p2 = polygon[i + 1];
    if (p.second > std::min(p1.second, p2.second) && p.second <= std::max(p1.second, p2.second) && p.first <= std::max(p1.first, p2.first) &&
        p1.second != p2.second) {
      const double x_intersection = (p.second - p1.second) * (p2.first - p1.first) / (p2.second - p1.second) + p1.first;
      if (p1.first == p2.first || p.first <= x_intersection) {
        intersections++;
      }
    }
  }
  return (intersections % 2) == 1;
}

// All zones must be either disjoint or fully nested; partially overlapping edges break the decomposition.
bool zones_partially_overlap(const std::vector<polygon_t>& polygons)
{
  for (size_t i = 0; i < polygons.size(); ++i) {
    for (size_t j = i + 1; j < polygons.size(); ++j) {
      const auto& a = polygons[i];
      const auto& b = polygons[j];
      int a_in_b = 0;
      for (size_t k = 0; k + 1 < a.size(); ++k) {
        a_in_b += is_inside(a[k], b) ? 1 : 0;
      }
      int b_in_a = 0;
      for (size_t k = 0; k + 1 < b.size(); ++k) {
        b_in_a += is_inside(b[k], a) ? 1 : 0;
      }
      if ((a_in_b > 0 && a_in_b < static_cast<int>(a.size()) - 1) || (b_in_a > 0 && b_in_a < static_cast<int>(b.size()) - 1)) {
        return true;
      }
      for (size_t ka = 0; ka + 1 < a.size(); ++ka) {
        for (size_t kb = 0; kb + 1 < b.size(); ++kb) {
          if (detail::segments_intersect({a[ka].first, a[ka].second, a[ka + 1].first, a[ka + 1].second},
                                         {b[kb].first, b[kb].second, b[kb + 1].first, b[kb + 1].second})) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

double path_length(const std::vector<point_t>& path)
{
  double d = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    d += std::hypot(path[i].first - path[i - 1].first, path[i].second - path[i - 1].second);
  }
  return d;
}

std::shared_ptr<PathCostCalculator> make_cost_calculator(const DroneCostModel& model, const std::shared_ptr<loggers::SimpleLogger>& logger)
{
  if (model.optimization_type == "time") {
    return std::make_shared<TimeCalculator>(model.time);
  }
  if (model.optimization_type == "energy") {
    return std::make_shared<EnergyCalculator>(model.energy, logger);
  }
  return nullptr;
}

PlanResult fail(PlanResult res, const std::string& message, const std::shared_ptr<loggers::SimpleLogger>& logger)
{
  res.success = false;
  res.message = message;
  res.drones.clear();
  logger->log_err("[coverage_planner_core] " + message);
  return res;
}

}  // namespace

PlanResult plan(const PlanRequest& req, std::shared_ptr<loggers::SimpleLogger> logger)
{
  if (!logger) {
    logger = std::make_shared<loggers::SimpleLogger>();
    logger->set_log_level(loggers::LOG_INFO);
  }

  PlanResult res;
  res.seed = req.seed.has_value() ? *req.seed : std::random_device{}();

  const size_t n = req.drones.size();
  if (n == 0) {
    return fail(res, "no drones in the request", logger);
  }
  if (req.search_areas.empty()) {
    return fail(res, "no search areas in the request", logger);
  }
  if (!req.end_slots.empty() && req.end_slots.size() != n) {
    return fail(res, "end_slots must be empty or have exactly one slot per drone", logger);
  }
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      if (req.drones[i].name == req.drones[j].name) {
        return fail(res, "duplicate drone name '" + req.drones[i].name + "'", logger);
      }
    }
  }

  const point_t origin{req.origin.lat, req.origin.lon};
  const double sweep_height = req.sweep_height;
  const double transit_height = sweep_height + 1.0;  // the solver's default transit level, see ShortestPathCalculator
  const double sweeping_step = req.sweeping_step > 0.0 ? req.sweeping_step : req.params.sweeping_step;

  // --- zones -----------------------------------------------------------------------------------------------------
  std::vector<polygon_t> fly_zones;
  for (const auto& z : req.search_areas) {
    if (z.polygon.size() < 3) {
      return fail(res, "a search area has fewer than 3 points", logger);
    }
    fly_zones.push_back(to_polygon(z.polygon));
  }
  std::vector<polygon_t> no_fly_zones;
  std::vector<std::pair<polygon_t, double>> hr_no_fly_zones;
  for (const auto& z : req.no_fly_zones) {
    if (z.polygon.size() < 3) {
      return fail(res, "a no-fly zone has fewer than 3 points", logger);
    }
    if (z.max_altitude <= 0.0) {
      no_fly_zones.push_back(to_polygon(z.polygon));
    } else if (sweep_height > z.max_altitude) {
      // the sweep already flies above this height-restricted zone, so it restricts nothing
      logger->log_info("[coverage_planner_core] sweep height " + std::to_string(sweep_height) + " m clears a height-restricted zone (ceiling " +
                       std::to_string(z.max_altitude) + " m); ignoring it");
    } else {
      hr_no_fly_zones.emplace_back(to_polygon(z.polygon), z.max_altitude);
    }
  }
  {
    std::vector<polygon_t> all = fly_zones;
    all.insert(all.end(), no_fly_zones.begin(), no_fly_zones.end());
    for (const auto& hr : hr_no_fly_zones) {
      all.push_back(hr.first);
    }
    if (zones_partially_overlap(all)) {
      return fail(res, "edges of search areas and no-fly zones must not overlap (zones must be disjoint or fully nested)", logger);
    }
  }

  // --- solver configuration ----------------------------------------------------------------------------------------
  drone_spec_t spec;
  spec.optimization_type = req.model.optimization_type;
  spec.max_single_path_cost = req.model.max_single_path_cost;
  if (spec.optimization_type == "time") {
    spec.time_config = req.model.time;
  } else if (spec.optimization_type == "energy") {
    spec.energy_config = req.model.energy;
  } else {
    return fail(res, "unknown optimization_type '" + spec.optimization_type + "'", logger);
  }

  // The solver plans every path from one common start point; use the centroid of the drones' starts.
  point_t centroid{0.0, 0.0};
  for (const auto& d : req.drones) {
    centroid.first += d.start.lat / static_cast<double>(n);
    centroid.second += d.start.lon / static_cast<double>(n);
  }

  algorithm_config_t cfg{};
  cfg.number_of_rotations = req.params.number_of_rotations;
  cfg.points_in_lat_lon = true;
  cfg.lat_lon_origin = origin;
  cfg.number_of_drones = static_cast<int>(n);
  cfg.sweeping_step = sweeping_step;
  cfg.sweeping_alt = sweep_height;
  cfg.decomposition_type = req.params.decomposition_type;
  cfg.min_sub_polygons_per_uav = req.params.min_sub_polygons_per_uav;
  cfg.start_pos = centroid;
  cfg.rotations_per_cell = req.params.rotations_per_cell;
  cfg.no_improvement_cycles_before_stop = req.params.no_improvement_cycles_before_stop;
  cfg.drones.assign(n, spec);
  cfg.seed = res.seed;
  cfg.time_budget_s = req.params.time_budget_s;

  std::vector<std::shared_ptr<PathCostCalculator>> cost_calculators;
  for (size_t i = 0; i < n; ++i) {
    cost_calculators.push_back(make_cost_calculator(req.model, logger));
  }

  // One polygon holding all obstacles, used for paths between areas. Must outlive the ShortestPathCalculator.
  polygon_t empty_fly_zone;
  const MapPolygon master_obstacle_polygon(empty_fly_zone, no_fly_zones, origin, hr_no_fly_zones);
  const ShortestPathCalculator shortest_path_calculator(master_obstacle_polygon, true, sweep_height);

  // One MapPolygon per search area; only the no-fly zones inside it matter for decomposition and sweeping.
  std::vector<MapPolygon> search_areas;
  for (auto& fz : fly_zones) {
    MapPolygon area(fz, no_fly_zones, origin, hr_no_fly_zones);
    std::vector<polygon_t> internal_nfz;
    for (const auto& nfz : area.no_fly_zone_polygons) {
      if (!nfz.empty() && is_point_in_polygon(nfz[0], area.fly_zone_polygon_points)) {
        internal_nfz.push_back(nfz);
      }
    }
    area.no_fly_zone_polygons = internal_nfz;
    std::vector<HeightRestrictedNoFlyZone> internal_hr;
    for (const auto& hr : area.height_restricted_no_fly_zone_polygons) {
      if (!hr.polygon.empty() && is_point_in_polygon(hr.polygon[0], area.fly_zone_polygon_points)) {
        internal_hr.push_back(hr);
      }
    }
    area.height_restricted_no_fly_zone_polygons = internal_hr;
    search_areas.push_back(area);
  }

  // --- solve --------------------------------------------------------------------------------------------------------
  mstsp_solver::final_solution_t solution;
  try {
    solution = solve_for_uavs(static_cast<int>(n), cfg, search_areas, cost_calculators, shortest_path_calculator, logger);
  }
  catch (const std::exception& e) {
    return fail(res, std::string("solver failed: ") + e.what(), logger);
  }
  if (solution.paths.size() != n) {
    return fail(res, "solver returned " + std::to_string(solution.paths.size()) + " paths for " + std::to_string(n) + " drones", logger);
  }

  // Solver paths are [common start, (descent, sweep..., ascent)..., common start]: drop the common start at both ends.
  // A path left empty means the solver gave that drone no area.
  std::vector<path_t> sweeps(n);
  for (size_t j = 0; j < n; ++j) {
    auto& p = solution.paths[j];
    if (p.size() > 2) {
      sweeps[j].assign(p.begin() + 1, p.end() - 1);
    } else {
      logger->log_warn("[coverage_planner_core] the solver assigned no area to one of the paths (too slow or too costly drone model?)");
    }
  }

  // --- assign solved paths to drones ---------------------------------------------------------------------------------
  std::vector<point_t> starts(n);
  for (size_t i = 0; i < n; ++i) {
    starts[i] = gps_coordinates_to_meters({req.drones[i].start.lat, req.drones[i].start.lon}, origin);
  }
  const auto xy = [](const point_heading_t<double>& p) { return point_t{p.x, p.y}; };
  const auto transit_len = [&](const point_t& a, const point_t& b) { return path_length(shortest_path_calculator.shortest_path_between_points(a, b).first); };

  std::vector<std::vector<double>> start_cost(n, std::vector<double>(n, 0.0));
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (sweeps[j].empty()) {
        continue;  // any drone may take an empty path; let the other columns decide
      }
      start_cost[i][j] = transit_len(starts[i], xy(sweeps[j].front()));
      if (req.end_slots.empty() && req.return_to_start) {
        start_cost[i][j] += transit_len(xy(sweeps[j].back()), starts[i]);  // the drone also returns to its start
      }
    }
  }
  const std::vector<int> path_of_drone = detail::hungarian(start_cost);

  std::vector<point_t> ends = starts;
  std::vector<std::optional<size_t>> end_slot_of_drone(n);
  if (!req.end_slots.empty()) {
    std::vector<point_t> slots(n);
    for (size_t k = 0; k < n; ++k) {
      slots[k] = gps_coordinates_to_meters({req.end_slots[k].lat, req.end_slots[k].lon}, origin);
    }
    std::vector<std::vector<double>> end_cost(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
      const auto& s = sweeps[path_of_drone[i]];
      const point_t from = s.empty() ? starts[i] : xy(s.back());
      for (size_t k = 0; k < n; ++k) {
        end_cost[i][k] = transit_len(from, slots[k]);
      }
    }
    const std::vector<int> slot_of_drone = detail::hungarian(end_cost);
    for (size_t i = 0; i < n; ++i) {
      ends[i] = slots[slot_of_drone[i]];
      end_slot_of_drone[i] = static_cast<size_t>(slot_of_drone[i]);
    }
  }

  // --- full per-drone paths: start transit + sweep + end transit -------------------------------------------------------
  const auto append_transit = [](path_t& out, const std::vector<point_t>& pts, double height) {
    for (const auto& p : pts) {
      point_heading_t<double> wp(p.first, p.second, height);
      out.push_back(wp);
    }
  };
  std::vector<path_t> paths(n);
  // [begin, end) of the sweep part of each path. Recorded before transit-height resolution, which may lower
  // conflict-free transits to the sweep altitude, so altitude cannot tell sweep and transit apart afterwards.
  std::vector<std::pair<size_t, size_t>> sweep_range(n, {0, 0});
  for (size_t i = 0; i < n; ++i) {
    const auto& sweep = sweeps[path_of_drone[i]];
    auto& out = paths[i];
    if (sweep.empty()) {
      const auto t = shortest_path_calculator.shortest_path_between_points(starts[i], ends[i]);
      append_transit(out, t.first, t.second);
      continue;
    }
    auto t = shortest_path_calculator.shortest_path_between_points(starts[i], xy(sweep.front()));
    auto pts = t.first;
    if (!pts.empty()) {
      pts.pop_back();  // the sweep's own descent point continues from here
    }
    append_transit(out, pts, t.second);
    {
      // the solver's sweep segment is [descent point, lanes..., ascent point]; lanes are the points at sweep altitude
      size_t first = sweep.size(), last = 0;
      for (size_t k = 0; k < sweep.size(); ++k) {
        if (sweep[k].z == sweep_height) {
          first = std::min(first, k);
          last = k;
        }
      }
      sweep_range[i] = first < sweep.size() ? std::make_pair(out.size() + first, out.size() + last + 1) : std::make_pair(out.size(), out.size());
    }
    out.insert(out.end(), sweep.begin(), sweep.end());
    if (req.end_slots.empty() && !req.return_to_start) {
      continue;  // the path ends where the sweep ends
    }
    t = shortest_path_calculator.shortest_path_between_points(xy(sweep.back()), ends[i]);
    pts = t.first;
    if (!pts.empty()) {
      pts.erase(pts.begin());
    }
    append_transit(out, pts, t.second);
  }

  std::vector<double> min_h(n), min_v(n);
  for (size_t i = 0; i < n; ++i) {
    min_h[i] = req.drones[i].min_horizontal_distance;
    min_v[i] = req.drones[i].min_vertical_distance;
  }
  detail::resolve_transit_heights(paths, sweep_height, transit_height, min_h, min_v);

  // --- checks and output --------------------------------------------------------------------------------------------
  res.drones.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto& path = paths[i];
    auto& out = res.drones[i];
    out.name = req.drones[i].name;
    out.has_sweep = !sweeps[path_of_drone[i]].empty();
    out.end_slot = end_slot_of_drone[i];
    out.cost = cost_calculators[i] ? cost_calculators[i]->calculate_path_cost(path) : 0.0;

    if (out.cost > req.model.max_single_path_cost) {
      std::ostringstream ss;
      ss << "path of " << out.name << " costs " << out.cost << (req.model.optimization_type == "energy" ? " J" : " s") << ", above the limit of "
         << req.model.max_single_path_cost << "; more drones are needed";
      return fail(res, ss.str(), logger);
    }

    for (const auto& p : path) {
      if (req.max_altitude > 0.0 && p.z > req.max_altitude + 1e-6) {
        std::ostringstream ss;
        ss << "path of " << out.name << " needs altitude " << p.z << " m, above max_altitude " << req.max_altitude << " m";
        return fail(res, ss.str(), logger);
      }
    }
    out.sweep_begin = out.has_sweep ? sweep_range[i].first : path.size();
    out.sweep_end = out.has_sweep ? sweep_range[i].second : path.size();

    // Heading 0 on every waypoint, as in the fleet manager plugin (the solver's lane rotation angle is not used)
    out.waypoints.reserve(path.size());
    for (const auto& p : path) {
      const point_t ll = meters_to_gps_coordinates({p.x, p.y}, origin);
      out.waypoints.push_back(Waypoint{ll.first, ll.second, p.z, 0.0});
    }
  }

  res.success = true;
  res.message = "planned " + std::to_string(n) + " paths";
  return res;
}

}  // namespace coverage_planner_core
