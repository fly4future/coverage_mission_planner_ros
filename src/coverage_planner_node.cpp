#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Polygon.h>

#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>
#include <mrs_lib/param_loader.h>

#include <EnergyAwareMCPP/coverage_planner.hpp>
#include <mrs_coverage_planner/ComputeCoveragePath.h>
#include <mrs_coverage_planner/CoverageMission.h>
#include <mrs_coverage_planner/CoverageMissionRobot.h>
#include <mrs_coverage_planner/CoverageMissionGoal.h>
#include <mrs_coverage_planner/coverage_planner_core.h>
#include <mrs_coverage_planner/plugins/coverage_planner.h>

// Graph stores the information of transit paths overlapping. It is used in resolveTransitHeights function.
struct Graph {
    int V; // number of vertexes
    std::vector<std::vector<int>> adj; // List of neighbours

    Graph(int V) : V(V), adj(V) {}

    void addEdge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }
};

struct TransitPath
{
  double x1, y1, x2, y2;
  TransitPath(double x1, double y1, double x2, double y2) : x1(x1), y1(y1), x2(x2), y2(y2) {}
};

class TransitPathGroup
{
private:
  std::vector<std::unique_ptr<TransitPath>> transit_path_group;
  std::vector<double*> z_ptrs;
  
public:
  int drone_idx;

  double min_horizontal_distance;
  double min_vertical_distance;
  double drone_height;

  TransitPathGroup(int drone_idx, double min_horizontal_distance, double min_vertical_distance) 
      : drone_idx(drone_idx), min_horizontal_distance(min_horizontal_distance), min_vertical_distance(min_vertical_distance) {
  }

  const std::vector<std::unique_ptr<TransitPath>>& get() const {
    return transit_path_group;
  }

  void setHeight(double height, double sweeping_height, double transit_height) {
    if (height == transit_height) {
      drone_height = sweeping_height;
    } else {
      drone_height = height + min_vertical_distance;
    }
  }

  void writeTransitPathHeights(double height) {
    for (double* &z_ptr : z_ptrs) {
      if (z_ptr) *(z_ptr) = height;
    }
  }

  void addTransitPath(double x1, double y1, double x2, double y2, double *z1, double *z2) {
    std::unique_ptr<TransitPath> tp(new TransitPath(x1, y1, x2, y2));
    transit_path_group.push_back(std::move(tp));
    for (double* z_ptr : {z1, z2}) {
      if (std::find(z_ptrs.begin(), z_ptrs.end(), z_ptr) == z_ptrs.end()) {
        z_ptrs.push_back(z_ptr);
      }
    }
  }
};

struct TransitPathGroupsStruct
{
  std::vector<std::unique_ptr<TransitPathGroup>> transit_path_groups;
  std::vector<std::vector<int>> transit_paths_under;
};

struct NodePriority {
    int id;
    int degree;
    double best_available_height;

    bool operator>(const NodePriority& other) const {
        if (degree != other.degree) {
            return degree > other.degree;
        }
        return best_available_height < other.best_available_height;
    }
};

// Forward declarations for utility functions
bool checkForPotentialCycle(std::vector<std::vector<int>> &transit_paths_under, int starting_idx, int search_idx);
double distSq(mrs_coverage_planner::custom_types::Point2D p1, mrs_coverage_planner::custom_types::Point2D p2);
double pointToSegmentDistance(mrs_coverage_planner::custom_types::Point2D p, mrs_coverage_planner::custom_types::Point2D s1, mrs_coverage_planner::custom_types::Point2D s2);
std::vector<int> hungarianAlgorithm(const std::vector<std::vector<double>>& matrix);
std::vector<mrs_msgs::Reference> pointVecToWaypointVec(std::vector<mrs_coverage_planner::point_t> &points, double transit_path_height);
bool segmentsIntersect(TransitPath tp1, TransitPath tp2);
bool checkOverlap(TransitPath tp1, TransitPath tp2, double min_distance);
bool checkOverlap2(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double min_distance);
bool checkOverlap3(TransitPathGroup &tpg, TransitPath &tp, double min_distance);
int horizontalAndVerticalTPGIntersection(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double drone_distance);
bool is_inside(const mrs_coverage_planner::point_t& p, const std::vector<mrs_coverage_planner::point_t>& polygon);

class CoveragePlannerNode {
public:
    using coverage_paths_t = std::vector<std::vector<mrs_msgs::Reference>>;

    CoveragePlannerNode() : nh_("~") {
        mrs_lib::ParamLoader param_loader(nh_);
        param_loader.addYamlFile(ros::package::getPath("mrs_coverage_planner") + "/config/coverage_planner_config.yaml");
        planner_config_ = parse_algorithm_config(param_loader);
        if (!param_loader.loadedSuccessfully()) {
            ROS_ERROR("[CoveragePlannerNode]: Could not load all parameters!");
            exit(1);
        }
        
        service_ = nh_.advertiseService("compute_coverage_path", &CoveragePlannerNode::handleComputePathRequest, this);
        ROS_INFO("[CoveragePlannerNode]: Service initialized.");
    }

     

    coverage_paths_t getCoveragePaths(
        const mrs_coverage_planner::CoverageMission &mission,
        algorithm_config_t &config,
        const std::vector<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>> &fly_zones_arg,
        const std::vector<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>> &no_fly_zones_arg, 
        const std::vector<std::pair<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>, double>> &hr_no_fly_zones_arg,
        std::vector<double> min_horizontal_distances,
        std::vector<double> min_vertical_distances,
        double sweeping_height) const;

    void resolveTransitHeights(TransitPathGroupsStruct& tpgs, coverage_paths_t& coverage_paths, const Graph& graph, double sweeping_height, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances);

    bool handleComputePathRequest(mrs_coverage_planner::ComputeCoveragePath::Request &req,
                                 mrs_coverage_planner::ComputeCoveragePath::Response &res) {
        ROS_INFO("[CoveragePlannerNode]: Received incoming planning request. Projecting GPS frames...");

        try {

            // Create a request-local copy of the planner configuration
            algorithm_config_t config = planner_config_;

            // Override the world origin if supplied by the service request
            // Explicitly check that the coordinates are valid global GPS values and not unassigned zeros
            if (req.latitude_origin >= -90.0 && req.latitude_origin <= 90.0 && req.latitude_origin != 0.0 &&
                req.longitude_origin >= -180.0 && req.longitude_origin <= 180.0 && req.longitude_origin != 0.0) {
                config.lat_lon_origin.first  = req.latitude_origin;
                config.lat_lon_origin.second = req.longitude_origin;
            }
         // Priority 2: Fallback to the live global ROS parameter server (updated by your Python node at runtime)
            else {
                double param_lat, param_lon;
                if (ros::param::get("~latitude_origin", param_lat) && ros::param::get("~longitude_origin", param_lon)) {
                    config.lat_lon_origin.first  = param_lat;
                    config.lat_lon_origin.second = param_lon;
                }
            }

            ROS_INFO_STREAM("[CoveragePlannerNode]: Active Coordinate Origin Reference: "
                            << config.lat_lon_origin.first << ", " << config.lat_lon_origin.second);

            // 2. Parse Initial Drone Positions (Input GPS: x = Lat, y = Lon)
            mrs_coverage_planner::CoverageMission mission_msg;
            for (const auto& p : req.initial_drone_positions) {
                // pass pair as {lat, lon} matching your exact pipeline assignment
                auto projected_pt = gps_coordinates_to_meters({p.x, p.y}, config.lat_lon_origin);
                
                geometry_msgs::Point ros_pt;
                ros_pt.x = projected_pt.first;
                ros_pt.y = projected_pt.second;
                ros_pt.z = p.z;
                mission_msg.initial_positions.push_back(ros_pt);
            }

            // 3. Map Fly Zones (Input geometry_msgs/Polygon: points are Point32, x = Lat, y = Lon)
            std::vector<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>> search_areas_latlon;
            for (const auto& zone : req.fly_zones) {
                std::vector<mrs_coverage_planner::custom_types::Point2DLatLon> current_zone;
                for (const auto& p : zone.points) {
                    current_zone.push_back({static_cast<double>(p.x), static_cast<double>(p.y)}); 
                }
                if (!current_zone.empty()) {
                    search_areas_latlon.push_back(current_zone);
                }
            }

            // 4. Map No-Fly Zones
            std::vector<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>> no_fly_zones_latlon;
            for (const auto& poly : req.no_fly_zones) {
                std::vector<mrs_coverage_planner::custom_types::Point2DLatLon> current_nfz;
                for (const auto& p : poly.points) {
                    current_nfz.push_back({static_cast<double>(p.x), static_cast<double>(p.y)});
                }
                if (!current_nfz.empty()) {
                    no_fly_zones_latlon.push_back(current_nfz);
                }
            }

            // 5. Map Height Restricted (HR) No-Fly Zones
            std::vector<std::pair<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>, double>> hr_no_fly_zones_latlon;
            size_t hr_zones_count = std::min(req.hr_no_fly_zones.size(), req.hr_no_fly_depths.size());
            for (size_t i = 0; i < hr_zones_count; ++i) {
                std::vector<mrs_coverage_planner::custom_types::Point2DLatLon> current_hr_zone;
                for (const auto& p : req.hr_no_fly_zones[i].points) {
                    current_hr_zone.push_back({static_cast<double>(p.x), static_cast<double>(p.y)});
                }
                if (!current_hr_zone.empty()) {
                    hr_no_fly_zones_latlon.push_back({current_hr_zone, req.hr_no_fly_depths[i]});
                }
            }

            // 6. Balance Separation Arrays 
            std::vector<double> min_horiz = req.min_horizontal_distances;
            std::vector<double> min_vert = req.min_vertical_distances;
            size_t num_drones = req.initial_drone_positions.size();

            if (min_horiz.size() < num_drones) {
                min_horiz.resize(num_drones, min_horiz.empty() ? 5.0 : min_horiz.back());
            }
            if (min_vert.size() < num_drones) {
                min_vert.resize(num_drones, min_vert.empty() ? 5.0 : min_vert.back());
            }

             // If the request provides a valid sweeping step, override the YAML static value
            if (req.sweeping_step > 0.0) {
                config.sweeping_step = req.sweeping_step;
                ROS_INFO("[CoveragePlannerNode]: Overriding sweeping_step from request: %.2f meters", config.sweeping_step);
            }

            double sweeping_height = 0.0; 
            if (req.sweeping_height > 0.0) {
                sweeping_height = req.sweeping_height;
                ROS_INFO("[CoveragePlannerNode]: Overriding sweeping_height from request: %.2f meters", sweeping_height);
            } else {
                // Fallback to whatever safe default your system expects if omitted
                sweeping_height = 5.0; 
            }

            // Apply it directly to the configuration block used by EnergyAwareMCPP
            config.sweeping_alt = sweeping_height;
            // double transit_path_height = sweeping_height + 1.0;

            

            // 7. Execute Core Optimizer Calculation
            coverage_paths_t computed_paths = getCoveragePaths(
                mission_msg,
                config,
                search_areas_latlon,
                no_fly_zones_latlon,
                hr_no_fly_zones_latlon,
                min_horiz,
                min_vert,
                sweeping_height
            );

            if (computed_paths.empty()) {
                res.success = false;
                res.message = "Multi-UAV planner generated an empty optimization matrix.";
                return true;
            }

            // 8. Format Output Response to match Python's expectations exactly:
            // Python reads: pt.position.x -> passed to 'lat', pt.position.y -> passed to 'lon'
            for (const auto& single_drone_path : computed_paths) {
                mrs_msgs::TrajectoryReference trajectory;
                trajectory.header.stamp = ros::Time::now();
                trajectory.header.frame_id = "gps";
                trajectory.fly_now = true;
                trajectory.use_heading = true;

                for (const auto& wp : single_drone_path) {
                    mrs_msgs::Reference reference;
                    
                    // CRITICAL PAIR MATCH:
                    reference.position.x = wp.position.x; // Stores Latitude -> Python picks up as pt.position.x
                    reference.position.y = wp.position.y; // Stores Longitude -> Python picks up as pt.position.y
                    reference.position.z = wp.position.z; // Stores Altitude
                    reference.heading = wp.heading;
                    
                    trajectory.points.push_back(reference);
                }
                res.drone_paths.push_back(trajectory);
            }

            res.success = true;
            res.message = "Collision-free tracks successfully computed and exported to mission folder.";
        } 
        catch (const std::exception& e) {
            ROS_ERROR("[CoveragePlannerNode]: Coordinated planning pipeline crash: %s", e.what());
            res.success = false;
            res.message = std::string("Coordinated planning pipeline crash: ") + e.what();
        }

        return true;
    }
    
    algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const {
        const std::string yaml_prefix = "fleet_manager/planners/coverage_planner/";
        algorithm_config_t algorithm_config;

        param_loader.loadParam(yaml_prefix + "drone_mass", algorithm_config.energy_calculator_config.drone_mass);
        param_loader.loadParam(yaml_prefix + "drone_area", algorithm_config.energy_calculator_config.drone_area);
        param_loader.loadParam(yaml_prefix + "average_acceleration", algorithm_config.energy_calculator_config.average_acceleration);
        param_loader.loadParam(yaml_prefix + "propeller_radius", algorithm_config.energy_calculator_config.propeller_radius);
        param_loader.loadParam(yaml_prefix + "number_of_propellers", algorithm_config.energy_calculator_config.number_of_propellers);
        param_loader.loadParam(yaml_prefix + "allowed_path_deviation", algorithm_config.energy_calculator_config.allowed_path_deviation);
        param_loader.loadParam(yaml_prefix + "number_of_rotations", algorithm_config.number_of_rotations);

        const std::string battery_prefix = yaml_prefix + "battery_model/";
        param_loader.loadParam(battery_prefix + "cell_capacity", algorithm_config.energy_calculator_config.battery_model.cell_capacity);
        param_loader.loadParam(battery_prefix + "number_of_cells", algorithm_config.energy_calculator_config.battery_model.number_of_cells);
        param_loader.loadParam(battery_prefix + "d0", algorithm_config.energy_calculator_config.battery_model.d0);
        param_loader.loadParam(battery_prefix + "d1", algorithm_config.energy_calculator_config.battery_model.d1);
        param_loader.loadParam(battery_prefix + "d2", algorithm_config.energy_calculator_config.battery_model.d2);
        param_loader.loadParam(battery_prefix + "d3", algorithm_config.energy_calculator_config.battery_model.d3);

        const std::string speed_prefix = yaml_prefix + "best_speed_model/";
        param_loader.loadParam(speed_prefix + "c0", algorithm_config.energy_calculator_config.best_speed_model.c0);
        param_loader.loadParam(speed_prefix + "c1", algorithm_config.energy_calculator_config.best_speed_model.c1);
        param_loader.loadParam(speed_prefix + "c2", algorithm_config.energy_calculator_config.best_speed_model.c2);

        param_loader.loadParam(yaml_prefix + "points_in_lat_lon", algorithm_config.points_in_lat_lon);
        if (algorithm_config.points_in_lat_lon) {
            param_loader.loadParam(yaml_prefix + "latitude_origin", algorithm_config.lat_lon_origin.first);
            param_loader.loadParam(yaml_prefix + "longitude_origin", algorithm_config.lat_lon_origin.second);
        }

        param_loader.loadParam(yaml_prefix + "sweeping_step", algorithm_config.sweeping_step);

        int decomposition_method;
        param_loader.loadParam(yaml_prefix + "decomposition_method", decomposition_method);
        algorithm_config.decomposition_type = static_cast<decomposition_type_t>(decomposition_method);
        param_loader.loadParam(yaml_prefix + "min_sub_polygons_per_uav", algorithm_config.min_sub_polygons_per_uav);
        param_loader.loadParam(yaml_prefix + "rotations_per_cell", algorithm_config.rotations_per_cell);
        param_loader.loadParam(yaml_prefix + "no_improvement_cycles_before_stop", algorithm_config.no_improvement_cycles_before_stop);
        param_loader.loadParam(yaml_prefix + "max_single_path_energy", algorithm_config.max_single_path_energy);

        return algorithm_config;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    mutable algorithm_config_t planner_config_; 
};

// --- Utility Implementations ---

bool checkForPotentialCycle(std::vector<std::vector<int>> &transit_paths_under, int starting_idx, int search_idx) {
    for (int index : transit_paths_under.at(starting_idx)) {
        if (index == search_idx || checkForPotentialCycle(transit_paths_under, index, search_idx)) return true;
    }
    return false;
}

double distSq(mrs_coverage_planner::custom_types::Point2D p1, mrs_coverage_planner::custom_types::Point2D p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

double pointToSegmentDistance(mrs_coverage_planner::custom_types::Point2D p, mrs_coverage_planner::custom_types::Point2D s1, mrs_coverage_planner::custom_types::Point2D s2) {
    double l2 = distSq(s1, s2);
    if (l2 == 0.0) return std::sqrt(distSq(p, s1));
    double t = std::max(0.0, std::min(1.0, ((p.x - s1.x) * (s2.x - s1.x) + (p.y - s1.y) * (s2.y - s1.y)) / l2));
    mrs_coverage_planner::custom_types::Point2D projection = { s1.x + t * (s2.x - s1.x), s1.y + t * (s2.y - s1.y) };
    return std::sqrt(distSq(p, projection));
}

std::vector<int> hungarianAlgorithm(const std::vector<std::vector<double>>& matrix) {
    if (matrix.empty()) return {};
    int n = matrix.size(), m = matrix[0].size();
    const double INF_VAL = std::numeric_limits<double>::max();
    std::vector<double> u(n + 1, 0), v(m + 1, 0), minv(m + 1, 0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i; int j0 = 0;
        std::fill(minv.begin(), minv.end(), INF_VAL);
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0; double delta = INF_VAL;
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    double cur = matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else minv[j] -= delta;
            }
            j0 = j1;
        } while (p[j0] != 0);
        do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0 != 0);
    }
    std::vector<int> result(n);
    for (int j = 1; j <= m; ++j) if (p[j] != 0) result[p[j] - 1] = j - 1;
    return result;
}

std::vector<mrs_msgs::Reference> pointVecToWaypointVec(std::vector<mrs_coverage_planner::point_t> &points, double transit_path_height) {
    std::vector<mrs_msgs::Reference> waypoint_vec;
    for (auto &p : points) {
        mrs_msgs::Reference waypoint;
        waypoint.position.x = p.first;
        waypoint.position.y = p.second;
        waypoint.position.z = transit_path_height;
        waypoint.heading = 0.0;
        waypoint_vec.push_back(waypoint);
    }
    return waypoint_vec;
}

bool segmentsIntersect(TransitPath tp1, TransitPath tp2) {
    auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy) {
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax);
    };
    return (ccw(tp1.x1, tp1.y1, tp2.x1, tp2.y1, tp2.x2, tp2.y2) != ccw(tp1.x2, tp1.y2, tp2.x1, tp2.y1, tp2.x2, tp2.y2)) &&
           (ccw(tp1.x1, tp1.y1, tp1.x2, tp1.y2, tp2.x1, tp2.y1) != ccw(tp1.x1, tp1.y1, tp1.x2, tp1.y2, tp2.x2, tp2.y2));
}

bool checkOverlap(TransitPath tp1, TransitPath tp2, double min_distance) {
    if (segmentsIntersect(tp1, tp2)) return true;
    mrs_coverage_planner::custom_types::Point2D p1_1{tp1.x1, tp1.y1}, p1_2{tp1.x2, tp1.y2}, p2_1{tp2.x1, tp2.y1}, p2_2{tp2.x2, tp2.y2};
    double min_actual_dist = std::min({pointToSegmentDistance(p1_1, p2_1, p2_2), pointToSegmentDistance(p1_2, p2_1, p2_2), 
                                     pointToSegmentDistance(p2_1, p1_1, p1_2), pointToSegmentDistance(p2_2, p1_1, p1_2)});
    return min_actual_dist < min_distance;
}

bool checkOverlap2(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double min_distance) {
    for (const auto &tp1 : tpg1.get()) 
        for (const auto &tp2 : tpg2.get()) 
            if (checkOverlap(*tp1, *tp2, min_distance)) return true;
    return false;
}

bool checkOverlap3(TransitPathGroup &tpg, TransitPath &tp, double min_distance) {
    for (const auto &tp_ : tpg.get()) 
        if (checkOverlap(*tp_, tp, min_distance)) return true;
    return false;
}

int horizontalAndVerticalTPGIntersection(TransitPathGroup &tpg1, TransitPathGroup &tpg2, double drone_distance) {
    bool tpg1_above_tpg2 = false;
    mrs_coverage_planner::point_t start2 = {tpg2.get().front()->x1, tpg2.get().front()->y1}, end2 = {tpg2.get().back()->x2, tpg2.get().back()->y2};
    for (auto &tp : tpg1.get()) {
        if (pointToSegmentDistance({start2.first, start2.second}, {tp->x1, tp->y1}, {tp->x2, tp->y2}) < drone_distance ||
            pointToSegmentDistance({end2.first, end2.second}, {tp->x1, tp->y1}, {tp->x2, tp->y2}) < drone_distance) {
            tpg1_above_tpg2 = true; break;
        }
    }
    bool tpg2_above_tpg1 = false;
    mrs_coverage_planner::point_t start1 = {tpg1.get().front()->x1, tpg1.get().front()->y1}, end1 = {tpg1.get().back()->x2, tpg1.get().back()->y2};
    for (auto &tp : tpg2.get()) {
        if (pointToSegmentDistance({start1.first, start1.second}, {tp->x1, tp->y1}, {tp->x2, tp->y2}) < drone_distance ||
            pointToSegmentDistance({end1.first, end1.second}, {tp->x1, tp->y1}, {tp->x2, tp->y2}) < drone_distance) {
            tpg2_above_tpg1 = true; break;
        }
    }
    if (tpg1_above_tpg2 == tpg2_above_tpg1) return 0;
    return tpg1_above_tpg2 ? -1 : 1;
}

// Calculates the distance between a drone position and start and end of sweeping trajectory
double droneToSweepingDistance(point_t drone_pos, point_t start, point_t end, ShortestPathCalculator shortest_path_calculator)
{
  double distance = 0;
  std::vector<point_t> path_to_start = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {start.first, start.second}).first;
  for (int i = 1; i < path_to_start.size(); i++) {
    distance += std::sqrt(pow(path_to_start.at(i-1).first - path_to_start.at(i).first, 2) + pow(path_to_start.at(i-1).second - path_to_start.at(i).second, 2));
  }
  std::vector<point_t> path_to_end = shortest_path_calculator.shortest_path_between_points({drone_pos.first, drone_pos.second}, {end.first, end.second}).first;
  for (int i = 1; i < path_to_end.size(); i++) {
    distance += std::sqrt(pow(path_to_end.at(i-1).first - path_to_end.at(i).first, 2) + pow(path_to_end.at(i-1).second - path_to_end.at(i).second, 2));
  }
  return distance;
}

bool is_inside(const mrs_coverage_planner::point_t& p, const std::vector<mrs_coverage_planner::point_t>& polygon) {
    if (polygon.empty()) return false;
    int intersections = 0;
    for (size_t i = 0; i < polygon.size() - 1; ++i) {
        const auto& p1 = polygon[i]; const auto& p2 = polygon[i+1];
        if (p.second > std::min(p1.second, p2.second) && p.second <= std::max(p1.second, p2.second) &&
            p.first <= std::max(p1.first, p2.first) && p1.second != p2.second) {
            double x_int = (p.second - p1.second) * (p2.first - p1.first) / (p2.second - p1.second) + p1.first;
            if (p1.first == p2.first || p.first <= x_int) intersections++;
        }
    }
    return (intersections % 2) == 1;
}

void CoveragePlannerNode::resolveTransitHeights(TransitPathGroupsStruct& tpgs, coverage_paths_t& coverage_paths, const Graph& graph, double sweeping_height, std::vector<double> min_horizontal_distances, std::vector<double> min_vertical_distances) {
    int n = graph.V;
    if (n == 0) return;
    std::vector<double> assigned_heights(n, -1);
    std::vector<bool> processed(n, false);
    for (int i = 0; i < n; ++i) {
        int best_node = -1; NodePriority best_priority = {-1, -1, 1000000.0};
        for (int v = 0; v < n; ++v) {
            if (processed[v]) continue;
            bool skip = false;
            for (int tpg_idx : tpgs.transit_paths_under[v]) if (!processed[tpg_idx]) { skip = true; break; }
            if (skip) continue;
            double possible_height = std::max(sweeping_height, tpgs.transit_path_groups.at(v)->drone_height);
            for (int tpg_idx : tpgs.transit_paths_under[v]) 
                possible_height = std::max(possible_height, assigned_heights[tpg_idx] + std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, tpgs.transit_path_groups.at(tpg_idx)->min_vertical_distance));
            if (possible_height == sweeping_height) {
                for (int k = 0; k < (int)coverage_paths.size(); k++) {
                    if (k == tpgs.transit_path_groups.at(v)->drone_idx) continue;
                    for (size_t j = 1; j < coverage_paths.at(k).size(); j++) {
                        auto current_point = coverage_paths.at(k).at(j);
                        auto prev_point = coverage_paths.at(k).at(j-1);
                        if (current_point.position.z == sweeping_height && prev_point.position.z == sweeping_height) {
                            TransitPath tp(current_point.position.x, current_point.position.y, prev_point.position.x, prev_point.position.y);
                            if (checkOverlap3(*tpgs.transit_path_groups.at(v), tp, std::max(tpgs.transit_path_groups.at(v)->min_horizontal_distance, min_horizontal_distances.at(k))))
                                possible_height = std::max(possible_height, sweeping_height + std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, min_vertical_distances.at(k)));
                        }
                    }
                }
            }
            for (int neighbor : graph.adj[v]) {
                if (assigned_heights[neighbor] != -1 && std::abs(possible_height - assigned_heights[neighbor]) < std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, min_vertical_distances[tpgs.transit_path_groups.at(neighbor)->drone_idx]))
                    possible_height = assigned_heights[neighbor] + std::max(tpgs.transit_path_groups.at(v)->min_vertical_distance, min_vertical_distances[tpgs.transit_path_groups.at(neighbor)->drone_idx]);
            }
            NodePriority current = {v, (int)graph.adj[v].size(), possible_height};
            if (best_node == -1 || current > best_priority) { best_priority = current; best_node = v; }
        }
        if (best_node == -1) break;
        assigned_heights[best_node] = best_priority.best_available_height;
        processed[best_node] = true;
        tpgs.transit_path_groups.at(best_node)->drone_height = assigned_heights[best_node];
        tpgs.transit_path_groups.at(best_node)->writeTransitPathHeights(assigned_heights[best_node]);
    }
}



CoveragePlannerNode::coverage_paths_t CoveragePlannerNode::getCoveragePaths(
    const mrs_coverage_planner::CoverageMission &mission,
    algorithm_config_t &config, 
    const std::vector<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>> &fly_zones_arg, 
    const std::vector<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>> &no_fly_zones_arg, 
    const std::vector<std::pair<std::vector<mrs_coverage_planner::custom_types::Point2DLatLon>, double>> &hr_no_fly_zones_arg, 
    std::vector<double> min_horizontal_distances, 
    std::vector<double> min_vertical_distances,
    double sweeping_height) const 
{
    std::vector<mrs_coverage_planner::polygon_t> fly_zones, no_fly_zones;
    std::vector<std::pair<mrs_coverage_planner::polygon_t, double>> hr_no_fly_zones;

    for (const auto &sa : fly_zones_arg) {
        mrs_coverage_planner::polygon_t fz;
        for (const auto &p : sa) fz.emplace_back(p.lat, p.lon);
        if (!fz.empty() && fz.front() != fz.back()) fz.emplace_back(sa[0].lat, sa[0].lon);
        fly_zones.push_back(fz);
    }
    for (const auto &nz : no_fly_zones_arg) {
        mrs_coverage_planner::polygon_t nfz;
        for (const auto &p : nz) nfz.emplace_back(p.lat, p.lon);
        if (!nfz.empty() && nfz.front() != nfz.back()) nfz.emplace_back(nz[0].lat, nz[0].lon);
        no_fly_zones.push_back(nfz);
    }
    for (const auto &hr : hr_no_fly_zones_arg) {
        if (sweeping_height > hr.second) {
            ROS_INFO("[CoveragePlanner] Sweeping height (%.2f) clears HR ceiling (%.2f). Skipping restriction.", 
                     sweeping_height, hr.second);
            continue;
        }
        mrs_coverage_planner::polygon_t hnfz;
        for (const auto &p : hr.first) hnfz.emplace_back(p.lat, p.lon);
        if (!hnfz.empty() && hnfz.front() != hnfz.back()) hnfz.emplace_back(hr.first[0].lat, hr.first[0].lon);
        hr_no_fly_zones.emplace_back(std::move(hnfz), hr.second);
    }

    coverage_paths_t empty_paths;
    std::vector<mrs_coverage_planner::polygon_t> all_polygons;
    for (const auto &fz : fly_zones) all_polygons.push_back(fz);
    for (const auto &nfz : no_fly_zones) all_polygons.push_back(nfz);
    for (const auto &hr : hr_no_fly_zones) all_polygons.push_back(hr.first);

    // Validate: all zones must be either fully disjoint or fully contained
    for (size_t i = 0; i < all_polygons.size(); ++i) {
        for (size_t j = i + 1; j < all_polygons.size(); ++j) {
            bool overlap = false;
            int inside_i_j = 0; for (size_t a = 0; a + 1 < all_polygons[i].size(); ++a) if (is_inside(all_polygons[i][a], all_polygons[j])) inside_i_j++;
            int inside_j_i = 0; for (size_t b = 0; b + 1 < all_polygons[j].size(); ++b) if (is_inside(all_polygons[j][b], all_polygons[i])) inside_j_i++;
           
            // If some but not all vertices are inside, this is a partial overlap.
            if ((inside_i_j > 0 && inside_i_j < (int)all_polygons[i].size() - 1) || (inside_j_i > 0 && inside_j_i < (int)all_polygons[j].size() - 1)) overlap = true;
           
            // edge intersection check (covers crossings without vertex containment)
            for (size_t a = 0; a + 1 < all_polygons[i].size() && !overlap; ++a) {
                TransitPath s1(all_polygons[i][a].first, all_polygons[i][a].second, all_polygons[i][a+1].first, all_polygons[i][a+1].second);
                for (size_t b = 0; b + 1 < all_polygons[j].size() && !overlap; ++b) {
                    TransitPath s2(all_polygons[j][b].first, all_polygons[j][b].second, all_polygons[j][b+1].first, all_polygons[j][b+1].second);
                    if (segmentsIntersect(s1, s2)) overlap = true;
                }
            }
            if (overlap) { ROS_ERROR("[CoveragePlanner] Polygon overlap detected."); return empty_paths; }
        }
    }

    // double sweeping_height = 0.0; // Should come from request in real use
    double transit_path_height = sweeping_height + 1.0;

    config.number_of_drones = mission.initial_positions.size();
    config.sweeping_alt = sweeping_height;
    config.decomposition_type = BOUSTROPHEDON_DECOMPOSITION;
    config.min_sub_polygons_per_uav = 1;

    // Create a logger to log everything directly into stdout
    auto shared_logger = std::make_shared<loggers::SimpleLogger>();
    EnergyCalculator energy_calculator{config.energy_calculator_config, shared_logger};
    mrs_coverage_planner::polygon_t empty_fz;

    // Create one master polygon that contains ALL obstacles. This will be used for pathfinding between areas.
    // The fly-zone part is left empty, as the ShortestPathCalculator will ignore it anyway.
    MapPolygon master_obstacle_polygon = MapPolygon(empty_fz, no_fly_zones, config.lat_lon_origin, hr_no_fly_zones);
    ShortestPathCalculator shortest_path_calculator(master_obstacle_polygon, true, sweeping_height);

    // Now, create a vector of MapPolygon objects, one for each search area.
    // Each of these will contain its own fly zone boundary, but also ALL no-fly zones.
    // The trapezoidal decomposition will correctly handle only the NFZs inside the FZ.
    std::vector<MapPolygon> search_areas;
    for (polygon_t fly_zone : fly_zones) {
        MapPolygon area;//(fz, no_fly_zones, config.lat_lon_origin, hr_no_fly_zones);
        area = MapPolygon(fly_zone, no_fly_zones, config.lat_lon_origin, hr_no_fly_zones);

        // Remove outer no-fly zones only for decomposition and sweeping purposes for this specific area.
        // ShortestPathCalculator already loaded the original polygon with all zones for safe transit paths.
        std::vector<mrs_coverage_planner::polygon_t> internal_nfz;
        for (const auto& nfz : area.no_fly_zone_polygons) if (!nfz.empty() && is_point_in_polygon(nfz[0], area.fly_zone_polygon_points)) internal_nfz.push_back(nfz);
        area.no_fly_zone_polygons = internal_nfz;
        std::vector<HeightRestrictedNoFlyZone> internal_hr_nfz;
        for (const auto& hr_nfz : area.height_restricted_no_fly_zone_polygons) if (!hr_nfz.polygon.empty() && is_point_in_polygon(hr_nfz.polygon[0], area.fly_zone_polygon_points)) internal_hr_nfz.push_back(hr_nfz);
        
        area.height_restricted_no_fly_zone_polygons = internal_hr_nfz;
        search_areas.push_back(area);
    }
    
    // For saving the paths for each UAV
    coverage_paths_t coverage_paths_tmp;

    mstsp_solver::final_solution_t best_solution;
    try {
        config.start_pos = {mission.initial_positions.empty() ? 0.0 : mission.initial_positions[0].x, mission.initial_positions.empty() ? 0.0 : mission.initial_positions[0].y};
        auto solver_lambda = [&](int n) { return solve_for_uavs(n, config, search_areas, energy_calculator, shortest_path_calculator, shared_logger); };
        best_solution = generate_with_constraints(config.max_single_path_energy * 3600, config.number_of_drones, solver_lambda);
    } catch (const std::exception &e) { ROS_ERROR("[CoveragePlanner] Solver failed: %s", e.what()); return empty_paths; }

    // Save genrated path to coverage_paths_tmp excluding some points
    for (size_t d = 0; d < best_solution.paths.size(); ++d) {

        if (best_solution.paths.at(d).size() <= 2) continue;

        best_solution.paths.at(d).erase(best_solution.paths.at(d).begin());
        best_solution.paths.at(d).pop_back();
        std::vector<mrs_msgs::Reference> coverage_path;
        for (auto &p : best_solution.paths.at(d)) {
            mrs_msgs::Reference point; point.position.x = p.x; point.position.y = p.y; point.position.z = p.z; point.heading = 0.0;
            coverage_path.push_back(point);
        }
        coverage_paths_tmp.push_back(coverage_path);
        ROS_INFO("[CoveragePlanner] Drone %zu: Generated coverage path with %zu waypoints.", d, coverage_path.size());
        for (auto &wp : coverage_path) ROS_INFO("[CoveragePlanner] Waypoint: (%.2f, %.2f, %.2f)", wp.position.x, wp.position.y, wp.position.z);
    }

    // Get drone positions and start and end position of each sweeping path
    int drone_num = config.number_of_drones;
    if (coverage_paths_tmp.size() < (size_t)drone_num) return empty_paths;
    std::vector<mrs_coverage_planner::point_t> drone_pos(drone_num);
    std::vector<std::tuple<mrs_coverage_planner::point_t, mrs_coverage_planner::point_t>> path_ends(drone_num);
    for (int i = 0; i < drone_num; ++i) {
        //drone_pos.at(i) = gps_coordinates_to_meters({mission.initial_positions.at(i).x, mission.initial_positions.at(i).y}, config.lat_lon_origin);
        drone_pos.at(i) = {mission.initial_positions.at(i).x, mission.initial_positions.at(i).y};
        std::get<0>(path_ends.at(i)) = {coverage_paths_tmp.at(i).front().position.x, coverage_paths_tmp.at(i).front().position.y};
        std::get<1>(path_ends.at(i)) = {coverage_paths_tmp.at(i).back().position.x, coverage_paths_tmp.at(i).back().position.y};
    }

    // Create a matrix used for the hungarian algorithm
    std::vector<std::vector<double>> matrix(drone_num, std::vector<double>(drone_num, 0));
    for (int i = 0; i < drone_num; ++i) 
        for (int j = 0; j < drone_num; ++j) 
            matrix[i][j] = droneToSweepingDistance(drone_pos.at(i), std::get<0>(path_ends.at(j)), std::get<1>(path_ends.at(j)), shortest_path_calculator);

    // Hungarian algorithm finds the optimal global assignment of drones to sweeping paths to minimize total distance
    std::vector<int> assignment = hungarianAlgorithm(matrix);

    // coverage_paths is used to change order of the paths from coverage_paths_tmp. By changing the order of paths we assign each path to a different drone.
    coverage_paths_t coverage_paths(coverage_paths_tmp.size());
    TransitPathGroupsStruct tpgs;
    std::vector<double> min_horiz_tmp = min_horizontal_distances, min_vert_tmp = min_vertical_distances;

    for (int i = 0; i < drone_num; ++i) {
        // Saving coverage_paths_tmp into coverage_paths in different order. This way the path is assigned to a specific drone. We do this because we want to assign a coverage path to the nearest drone.
        coverage_paths.at(i) = coverage_paths_tmp.at(assignment[i]);

        // Calculates the path from the drone's starting position to the start of the sweeping path. If the direct route is obstructed by no-fly zones, shortest_path_calculator() finds a route around them.
        // std::vector<point_t> path_from_start = shortest_path_calculator.shortest_path_between_points({drone_positions.at(i).first, drone_positions.at(i).second}, {coverage_paths.at(i).at(1).reference.position.x, coverage_paths.at(i).at(1).reference.position.y});
        auto path_res = shortest_path_calculator.shortest_path_between_points({drone_pos.at(i).first, drone_pos.at(i).second}, {coverage_paths.at(i).at(1).position.x, coverage_paths.at(i).at(1).position.y});
        std::vector<mrs_coverage_planner::point_t> p_start = path_res.first;
        double h_transit = path_res.second;
        p_start.pop_back();
        
        auto start_wps = pointVecToWaypointVec(p_start, h_transit);
        coverage_paths.at(i).insert(coverage_paths.at(i).begin(), start_wps.begin(), start_wps.end());
        path_res = shortest_path_calculator.shortest_path_between_points({coverage_paths.at(i).back().position.x, coverage_paths.at(i).back().position.y}, {drone_pos.at(i).first, drone_pos.at(i).second});
        std::vector<mrs_coverage_planner::point_t> p_end = path_res.first;
        h_transit = path_res.second;
        p_end.erase(p_end.begin());
        auto end_wps = pointVecToWaypointVec(p_end, h_transit);
        coverage_paths.at(i).insert(coverage_paths.at(i).end(), end_wps.begin(), end_wps.end());
        
        // Fill the TransitPathGroupStruct
        for (size_t j = 1; j < coverage_paths.at(i).size(); ++j) {
            mrs_coverage_planner::custom_types::Point2D curr(coverage_paths.at(i).at(j).position.x, coverage_paths.at(i).at(j).position.y), prev(coverage_paths.at(i).at(j-1).position.x, coverage_paths.at(i).at(j-1).position.y);
            if (coverage_paths.at(i).at(j).position.z > sweeping_height && coverage_paths.at(i).at(j-1).position.z > sweeping_height && (curr.x != prev.x || curr.y != prev.y)) {
               
                // Checking if the current TransitPath (composed of current_point and prev_point) connects to the last added TransitPath.
                // If it does, the last TransitPathGroup is extended. If not, a new TransitPathGroup is created.
                if (!tpgs.transit_path_groups.empty() && tpgs.transit_path_groups.back()->drone_idx == i && tpgs.transit_path_groups.back()->get().back()->x2 == prev.x && tpgs.transit_path_groups.back()->get().back()->y2 == prev.y)
                    tpgs.transit_path_groups.back()->addTransitPath(prev.x, prev.y, curr.x, curr.y, &coverage_paths.at(i).at(j-1).position.z, &coverage_paths.at(i).at(j).position.z);
                else {
                    auto tpg = std::make_unique<TransitPathGroup>(i, min_horiz_tmp.at(i), min_vert_tmp.at(i));
                    tpg->addTransitPath(prev.x, prev.y, curr.x, curr.y, &coverage_paths.at(i).at(j-1).position.z, &coverage_paths.at(i).at(j).position.z);
                    tpg->setHeight(coverage_paths.at(i).at(j).position.z, sweeping_height, transit_path_height);
                    tpgs.transit_path_groups.push_back(std::move(tpg));
                }
            }
        }
    }

    // Fill the transit_paths_under vector.
    tpgs.transit_paths_under.insert(tpgs.transit_paths_under.end(), tpgs.transit_path_groups.size(), std::vector<int>());
    for (size_t i = 0; i < tpgs.transit_path_groups.size(); ++i) {
        for (size_t j = i+1; j < tpgs.transit_path_groups.size(); ++j) {
            if (tpgs.transit_path_groups.at(i)->drone_idx == tpgs.transit_path_groups.at(j)->drone_idx) continue;
            int r = horizontalAndVerticalTPGIntersection(*tpgs.transit_path_groups.at(i), *tpgs.transit_path_groups.at(j), std::max(min_horiz_tmp.at(tpgs.transit_path_groups.at(i)->drone_idx), min_horiz_tmp.at(tpgs.transit_path_groups.at(j)->drone_idx)));
            if (r == -1 && !checkForPotentialCycle(tpgs.transit_paths_under, j, i)) tpgs.transit_paths_under.at(i).push_back(j);
            else if (r == 1 && !checkForPotentialCycle(tpgs.transit_paths_under, i, j)) tpgs.transit_paths_under.at(j).push_back(i);
        }
    }
    
    // Graph is created. In this graph vertexes are transit path groups and edges mean that two transit path groups overlap
    Graph graph(tpgs.transit_path_groups.size());
    for (size_t i = 0; i < tpgs.transit_path_groups.size(); ++i) {
        for (size_t j = i+1; j < tpgs.transit_path_groups.size(); ++j) {
            if (tpgs.transit_path_groups.at(i)->drone_idx != tpgs.transit_path_groups.at(j)->drone_idx && checkOverlap2(*tpgs.transit_path_groups.at(i), *tpgs.transit_path_groups.at(j), std::max(min_horiz_tmp.at(tpgs.transit_path_groups.at(i)->drone_idx), min_horiz_tmp.at(tpgs.transit_path_groups.at(j)->drone_idx))))
                graph.addEdge(i, j);
        }
    }
    const_cast<CoveragePlannerNode*>(this)->resolveTransitHeights(tpgs, coverage_paths, graph, sweeping_height, min_horiz_tmp, min_vert_tmp);
    
    // Covert coverage_paths to gps coordinates
    for (int i = 0; i < drone_num; ++i) {
        for (size_t j = 0; j < coverage_paths.at(i).size(); ++j) {
            point_t d2 = meters_to_gps_coordinates({coverage_paths.at(i).at(j).position.x, coverage_paths.at(i).at(j).position.y}, config.lat_lon_origin);
            coverage_paths.at(i).at(j).position.x = d2.first; coverage_paths.at(i).at(j).position.y = d2.second;
        }
    }
    return coverage_paths;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mrs_coverage_planner_node");
    CoveragePlannerNode node;
    ros::spin();
    return 0;
}
