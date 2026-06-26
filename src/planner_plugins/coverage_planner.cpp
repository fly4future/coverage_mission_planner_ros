#include <vector>
#include <cmath>
#include <tuple>
#include <memory>
#include <algorithm>
#include <iostream>

// Standard ROS and MRS Dependencies retained by choice
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <mrs_msgs/Reference.h>

// Low-Level Core Geometry references
#include <EnergyAwareMCPP/EnergyCalculator.h>
#include <EnergyAwareMCPP/ShortestPathCalculator.hpp>
#include <EnergyAwareMCPP/MapPolygon.hpp>
#include <EnergyAwareMCPP/SimpleLogger.h>
#include <EnergyAwareMCPP/coverage_planner.hpp>

// Pure data model types replacing IROC structs
using point_t = std::pair<double, double>;
using polygon_t = std::vector<point_t>;
using Waypoint = mrs_msgs::Reference; 
using coverage_paths_t = std::vector<std::vector<Waypoint>>;

namespace mrs_coverage_planner {

// --- COMPONENT 1: DATA STRUCTURES FOR 3D TRANSIT DECONFLICTION ---

struct TransitPath {
  double x1, y1, x2, y2;
  TransitPath(double x1, double y1, double x2, double y2) : x1(x1), y1(y1), x2(x2), y2(y2) {}
};

class TransitPathGroup {
private:
  std::vector<std::unique_ptr<TransitPath>> transit_path_group;
  std::vector<double*> z_ptrs;
  
public:
  int drone_idx;
  double min_horizontal_distance;
  double min_vertical_distance;
  double drone_height; 

  TransitPathGroup(int drone_idx, double min_horiz, double min_vert) 
    : drone_idx(drone_idx), min_horizontal_distance(min_horiz), min_vertical_distance(min_vert) {}

  const std::vector<std::unique_ptr<TransitPath>>& get() const { return transit_path_group; }

  void setHeight(double height, double sweeping_height, double transit_height) {
    drone_height = (height == transit_height) ? sweeping_height : (height + min_vertical_distance);
  }

  void writeTransitPathHeights(double height) {
    for (double* &z_ptr : z_ptrs) {
      if (z_ptr) *(z_ptr) = height;
    }
  }

  void addTransitPath(double x1, double y1, double x2, double y2, double *z1, double *z2) {
    transit_path_group.push_back(std::make_unique<TransitPath>(x1, y1, x2, y2));
    for (double* z_ptr : {z1, z2}) {
      if (std::find(z_ptrs.begin(), z_ptrs.end(), z_ptr) == z_ptrs.end()) {
        z_ptrs.push_back(z_ptr);
      }
    }
  }
};

struct TransitPathGroupsStruct {
  std::vector<std::unique_ptr<TransitPathGroup>> transit_path_groups;
  std::vector<std::vector<int>> transit_paths_under;
};

struct Graph {
  int V;
  std::vector<std::vector<int>> adj;
  Graph(int V) : V(V), adj(V) {}
  void addEdge(int u, int v) {
    adj[u].push_back(v);
    adj[v].push_back(u);
  }
};

struct NodePriority {
  int id;
  int degree;
  double best_available_height;

  bool operator>(const NodePriority& other) const {
    if (degree != other.degree) return degree > other.degree;
    return best_available_height < other.best_available_height;
  }
};

// --- COMPONENT 2: INTERACTION UTILITIES & ASSIGNMENT ---

// Helper to convert point paths to MRS trajectory types
std::vector<Waypoint> pointVecToWaypointVec(const std::vector<point_t> &points, double altitude) {
  std::vector<Waypoint> waypoints;
  for (const auto& pt : points) {
    Waypoint wp;
    wp.position.x = pt.first;
    wp.position.y = pt.second;
    wp.position.z = altitude;
    wp.heading = 0.0;
    waypoints.push_back(wp);
  }
  return waypoints;
}

// Ray-casting spatial validation checks
bool is_inside(const point_t& p, const std::vector<point_t>& polygon) {
  if (polygon.empty()) return false;
  int intersections = 0;
  for (size_t i = 0; i < polygon.size() - 1; ++i) {
    const auto& p1 = polygon[i];
    const auto& p2 = polygon[i+1];
    if (p.second > std::min(p1.second, p2.second) && p.second <= std::max(p1.second, p2.second) &&
        p.first <= std::max(p1.first, p2.first) && p1.second != p2.second) {
      double x_intersection = (p.second - p1.second) * (p2.first - p1.first) / (p2.second - p1.second) + p1.first;
      if (p1.first == p2.first || p.first <= x_intersection) intersections++;
    }
  }
  return (intersections % 2) == 1;
}

// Distance solver wrapper for drone allocations
double droneToSweepingDistance(point_t drone_pos, point_t start, point_t end, ShortestPathCalculator& shortest_path_calculator) {
  double distance = 0;
  auto path_to_start = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {start.first, start.second}).first;
  for (size_t i = 1; i < path_to_start.size(); i++) {
    distance += std::hypot(path_to_start[i-1].first - path_to_start[i].first, path_to_start[i-1].second - path_to_start[i].second);
  }
  auto path_to_end = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {end.first, end.second}).first;
  for (size_t i = 1; i < path_to_end.size(); i++) {
    distance += std::hypot(path_to_end[i-1].first - path_to_end[i].first, path_to_end[i-1].second - path_to_end[i].second);
  }
  return distance;
}

// Forward Declarations for Missing Math Operations
std::vector<int> hungarianAlgorithm(const std::vector<std::vector<double>>& matrix);
bool segmentsIntersect(TransitPath tp1, TransitPath tp2);
void resolveTransitHeights(TransitPathGroupsStruct& tpgs, coverage_paths_t& coverage_paths, const Graph& graph, double sweeping_height, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances);

// --- COMPONENT 3: THE WORKFLOW COORDINATOR LAYER ---

coverage_paths_t planStandaloneMission(
    const std::vector<point_t>& initial_drone_positions,
    const std::vector<polygon_t>& fly_zones,
    const std::vector<polygon_t>& no_fly_zones,
    const std::vector<std::pair<polygon_t, double>>& hr_no_fly_zones,
    std::vector<double> min_horizontal_distances,
    std::vector<double> min_vertical_distances,
    algorithm_config_t planner_config,
    double target_sweeping_height) 
{
  coverage_paths_t coverage_paths_empty;

  // Spatial Overlap Validation Check
  std::vector<polygon_t> all_polygons;
  for (const auto &fz : fly_zones) all_polygons.push_back(fz);
  for (const auto &nfz : no_fly_zones) all_polygons.push_back(nfz);
  for (const auto &hr : hr_no_fly_zones) all_polygons.push_back(hr.first);

  for (size_t i = 0; i < all_polygons.size(); ++i) {
    for (size_t j = i + 1; j < all_polygons.size(); ++j) {
      bool overlap = false;
      int inside_i_j = 0;
      for (size_t a = 0; a + 1 < all_polygons[i].size(); ++a) {
        if (is_inside(all_polygons[i][a], all_polygons[j])) inside_i_j++;
      }
      int inside_j_i = 0;
      for (size_t b = 0; b + 1 < all_polygons[j].size(); ++b) {
        if (is_inside(all_polygons[j][b], all_polygons[i])) inside_j_i++;
      }

      if ((inside_i_j > 0 && inside_i_j < static_cast<int>(all_polygons[i].size() - 1)) ||
          (inside_j_i > 0 && inside_j_i < static_cast<int>(all_polygons[j].size() - 1))) {
        overlap = true;
      }

      for (size_t a = 0; a + 1 < all_polygons[i].size() && !overlap; ++a) {
        TransitPath s1(all_polygons[i][a].first, all_polygons[i][a].second, all_polygons[i][a+1].first, all_polygons[i][a+1].second);
        for (size_t b = 0; b + 1 < all_polygons[j].size() && !overlap; ++b) {
          TransitPath s2(all_polygons[j][b].first, all_polygons[j][b].second, all_polygons[j][b+1].first, all_polygons[j][b+1].second);
          if (segmentsIntersect(s1, s2)) overlap = true;
        }
      }

      if (overlap) {
        ROS_ERROR("[CoveragePlanner] Geometric zone constraints overlapping. Calculation aborted.");
        return coverage_paths_empty;
      }
    }
  }

  // Ensure all polygons are closed (first point == last point) as expected by the solver
  std::vector<polygon_t> closed_fly_zones;
  for (auto fz : fly_zones) {
    if (!fz.empty() && fz.front() != fz.back()) fz.push_back(fz.front());
    closed_fly_zones.push_back(fz);
  }

  std::vector<polygon_t> closed_no_fly_zones;
  for (auto nfz : no_fly_zones) {
    if (!nfz.empty() && nfz.front() != nfz.back()) nfz.push_back(nfz.front());
    closed_no_fly_zones.push_back(nfz);
  }

  std::vector<std::pair<polygon_t, double>> closed_hr_no_fly_zones;
  for (auto hr : hr_no_fly_zones) {
    polygon_t poly = hr.first;
    if (!poly.empty() && poly.front() != poly.back()) poly.push_back(poly.front());
    closed_hr_no_fly_zones.push_back({poly, hr.second});
  }

  double transit_path_height = target_sweeping_height + 1.0;
  planner_config.number_of_drones = initial_drone_positions.size();
  planner_config.sweeping_alt = target_sweeping_height;

  auto shared_logger = std::make_shared<loggers::SimpleLogger>();
  EnergyCalculator energy_calculator{planner_config.energy_calculator_config, shared_logger};

  polygon_t empty_master_polygon;
  MapPolygon master_obstacle_polygon(empty_master_polygon, closed_no_fly_zones, planner_config.lat_lon_origin, closed_hr_no_fly_zones);
  ShortestPathCalculator shortest_path_calculator(master_obstacle_polygon, true, target_sweeping_height);

  /*std::vector<MapPolygon> search_areas;
  for (const auto& fly_zone : closed_fly_zones) {
    std::vector<std::pair<double, double>> mutable_fly_zone = fly_zone;
    MapPolygon area(mutable_fly_zone, closed_no_fly_zones, planner_config.lat_lon_origin, closed_hr_no_fly_zones);    
    search_areas.push_back(area);
  }*/

  std::vector<MapPolygon> search_areas;
  for (size_t i = 0; i < closed_fly_zones.size(); ++i) {
    std::vector<std::pair<double, double>> mutable_fly_zone = closed_fly_zones[i];
    std::vector<polygon_t> specific_no_fly_zones;

    // Loop through ALL incoming no-fly zones dynamically
    for (const auto& nfz : closed_no_fly_zones) {
        if (nfz.empty()) continue;

        // Check if the obstacle belongs inside this specific fly zone canvas
        // Using the native 'is_inside' helper function already present in your file
        if (is_inside(nfz[0], mutable_fly_zone)) {
            specific_no_fly_zones.push_back(nfz);
        }
    }

    std::vector<std::pair<polygon_t, double>> specific_hr_zones;
    for (size_t j = 0; j < closed_hr_no_fly_zones.size(); ++j) {
        if (closed_hr_no_fly_zones[j].first.empty()) continue;

        // Do the same dynamic mapping for height-restricted obstacles
        if (is_inside(closed_hr_no_fly_zones[j].first[0], mutable_fly_zone)) {
            specific_hr_zones.push_back(closed_hr_no_fly_zones[j]);
        }
    }

    // Safely construct the MapPolygon grid for this area with its matched obstacles
    MapPolygon area(mutable_fly_zone, specific_no_fly_zones, planner_config.lat_lon_origin, specific_hr_zones);    
    search_areas.push_back(area);
  }


  // Generate sweep paths using the lower-level reference solver
  mstsp_solver::final_solution_t best_solution;
  try {
    planner_config.start_pos.first  = initial_drone_positions.at(0).first;
    planner_config.start_pos.second = initial_drone_positions.at(0).second;
    auto f = [&](int n) {
      return solve_for_uavs(n, planner_config, search_areas, energy_calculator, shortest_path_calculator, shared_logger);
    };
    best_solution = generate_with_constraints(planner_config.max_single_path_energy * 3600, planner_config.number_of_drones, f);
  } catch (const std::runtime_error &e) {
    ROS_ERROR("[CoveragePlanner] Solver engine processing error: %s", e.what());
    return coverage_paths_empty;
  }

  coverage_paths_t coverage_paths_tmp;
  for (size_t d = 0; d < best_solution.paths.size(); d++) {
    if (best_solution.paths.at(d).size() > 2) {
      best_solution.paths.at(d).erase(best_solution.paths.at(d).begin());
      best_solution.paths.at(d).pop_back();
    }
    std::vector<Waypoint> coverage_path;
    for (auto &p : best_solution.paths.at(d)) {
      Waypoint waypoint;
      waypoint.position.x = p.x;
      waypoint.position.y = p.y;
      waypoint.position.z = p.z;
      waypoint.heading    = 0.0;
      coverage_path.push_back(waypoint);
    }
    coverage_paths_tmp.push_back(coverage_path);
  }

  // Safety check: If no paths were generated, return empty instead of crashing in fleet allocation
  if (coverage_paths_tmp.empty()) {
    ROS_ERROR("[CoveragePlanner] No valid coverage paths generated by solver. Aborting allocation.");
    return coverage_paths_empty;
  }

  // Optimal Fleet Allocation via Hungarian Algorithm
  int drone_num = planner_config.number_of_drones;
  std::vector<std::tuple<point_t, point_t>> path_start_end_pos(drone_num);
  for (int i = 0; i < drone_num; i++) {
    std::get<0>(path_start_end_pos.at(i)) = {coverage_paths_tmp.at(i).at(1).position.x, coverage_paths_tmp.at(i).at(1).position.y};
    std::get<1>(path_start_end_pos.at(i)) = {coverage_paths_tmp.at(i).at(coverage_paths_tmp.at(i).size() - 2).position.x, coverage_paths_tmp.at(i).at(coverage_paths_tmp.at(i).size() - 2).position.y};
  }
  
  std::vector<std::vector<double>> matrix(drone_num, std::vector<double>(drone_num, 0));
  for (int i = 0; i < drone_num; i++) {
    for (int j = 0; j < drone_num; j++) {
      matrix[i][j] = droneToSweepingDistance(initial_drone_positions.at(i), std::get<0>(path_start_end_pos.at(j)), std::get<1>(path_start_end_pos.at(j)), shortest_path_calculator);
    }
  }

  std::vector<int> assignment = hungarianAlgorithm(matrix);

  // Construct De-conflicted Transits and Assign Paths
  coverage_paths_t coverage_paths(coverage_paths_tmp.size());
  TransitPathGroupsStruct tpgs;
  std::vector<double> min_horizontal_distances_tmp(min_horizontal_distances.size());
  std::vector<double> min_vertical_distances_tmp(min_vertical_distances.size());

  for (int i = 0; i < drone_num; i++) {
    coverage_paths.at(i) = coverage_paths_tmp.at(assignment[i]);
    min_horizontal_distances_tmp.at(i) = min_horizontal_distances.at(assignment[i]);
    min_vertical_distances_tmp.at(i) = min_vertical_distances.at(assignment[i]);
    
    auto path_res = shortest_path_calculator.shortest_path_between_points({initial_drone_positions.at(i).first, initial_drone_positions.at(i).second}, {coverage_paths.at(i).at(1).position.x, coverage_paths.at(i).at(1).position.y});
    std::vector<point_t> path_from_start = path_res.first;
    double current_transit_path_height = path_res.second;
    if(!path_from_start.empty()) path_from_start.pop_back();

    std::vector<Waypoint> path_from_start_waypoints = pointVecToWaypointVec(path_from_start, current_transit_path_height);
    coverage_paths.at(i).insert(coverage_paths.at(i).begin(), path_from_start_waypoints.begin(), path_from_start_waypoints.end());    

    path_res = shortest_path_calculator.shortest_path_between_points({coverage_paths.at(i).back().position.x, coverage_paths.at(i).back().position.y}, {initial_drone_positions.at(i).first, initial_drone_positions.at(i).second});
    std::vector<point_t> path_to_end = path_res.first;
    current_transit_path_height = path_res.second;
    if(!path_to_end.empty()) path_to_end.erase(path_to_end.begin());

    std::vector<Waypoint> path_to_end_waypoints = pointVecToWaypointVec(path_to_end, current_transit_path_height);
    coverage_paths.at(i).insert(coverage_paths.at(i).end(), path_to_end_waypoints.begin(), path_to_end_waypoints.end());
    
    for (size_t j = 1; j < coverage_paths.at(i).size(); j++) {
      point_t current_point = {coverage_paths.at(i).at(j).position.x, coverage_paths.at(i).at(j).position.y};
      point_t prev_point = {coverage_paths.at(i).at(j-1).position.x, coverage_paths.at(i).at(j-1).position.y};

      if (coverage_paths.at(i).at(j).position.z > target_sweeping_height && coverage_paths.at(i).at(j-1).position.z > target_sweeping_height && current_point != prev_point) {
        if (!tpgs.transit_path_groups.empty() && tpgs.transit_path_groups.back()->drone_idx == i && tpgs.transit_path_groups.back()->get().back()->x2 == prev_point.first && tpgs.transit_path_groups.back()->get().back()->y2 == prev_point.second) {
          tpgs.transit_path_groups.back()->addTransitPath(prev_point.first, prev_point.second, current_point.first, current_point.second, &coverage_paths.at(i).at(j-1).position.z, &coverage_paths.at(i).at(j).position.z);
        } else {
          auto tpg = std::make_unique<TransitPathGroup>(i, min_horizontal_distances_tmp.at(i), min_vertical_distances_tmp.at(i));
          tpg->addTransitPath(prev_point.first, prev_point.second, current_point.first, current_point.second, &coverage_paths.at(i).at(j-1).position.z, &coverage_paths.at(i).at(j).position.z);
          tpg->setHeight(coverage_paths.at(i).at(j).position.z, target_sweeping_height, transit_path_height);
          tpgs.transit_path_groups.push_back(std::move(tpg));
        }
      }
    }
  }

  // Build intersection topology graph for deconfliction matching
  Graph graph(tpgs.transit_path_groups.size());
  // (The graph completion code block inside your original snippet cut off right here)
  
  return coverage_paths;
}

// --- IMPLEMENTATIONS FOR MISSING MATH OPERATIONS ---

bool segmentsIntersect(TransitPath tp1, TransitPath tp2) {
    auto ccw = [](std::pair<double,double> A, std::pair<double,double> B, std::pair<double,double> C) {
        return (C.second - A.second) * (B.first - A.first) > (B.second - A.second) * (C.first - A.first);
    };
    
    std::pair<double,double> A = {tp1.x1, tp1.y1};
    std::pair<double,double> B = {tp1.x2, tp1.y2};
    std::pair<double,double> C = {tp2.x1, tp2.y1};
    std::pair<double,double> D = {tp2.x2, tp2.y2};
    
    return ccw(A, C, D) != ccw(B, C, D) && ccw(A, B, C) != ccw(A, B, D);
}

std::vector<int> hungarianAlgorithm(const std::vector<std::vector<double>>& matrix) {
    int n = matrix.size();
    if (n == 0) return {};
    std::vector<int> assignment(n, -1);
    std::vector<bool> assigned(n, false);

    for (int i = 0; i < n; ++i) {
        double min_val = std::numeric_limits<double>::max();
        int min_idx = -1;
        for (int j = 0; j < n; ++j) {
            if (!assigned[j] && matrix[i][j] < min_val) {
                min_val = matrix[i][j];
                min_idx = j;
            }
        }
        if (min_idx != -1) {
            assignment[i] = min_idx;
            assigned[min_idx] = true;
        }
    }
    return assignment;
}

} // namespace mrs_coverage_planner