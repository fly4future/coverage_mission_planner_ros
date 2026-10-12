/* include and declarations //{ */

#include "PathCostCalculator.hpp"
#include "MapPolygon.hpp"
#include "EnergyCalculator.h"
#include "TimeCalculator.hpp"
#include "algorithms.hpp"
#include "ShortestPathCalculator.hpp"
#include "mstsp_solver/SolverConfig.h"
#include "mstsp_solver/MstspSolver.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "SimpleLogger.h"
#include "utils.hpp"
#include <iomanip>
#include <coverage_planner.hpp>


/*!
 * Parse algorithm configuration from YAML object into algorithm_config_t object
 * @param config YAML node read from the configuration file
 */
algorithm_config_t parse_algorithm_config(const YAML::Node& config);

/*!
 * Check whether the algorithm configuration is valid (i.e. contains all the required fields and dependencies)
 */
bool algorithm_config_is_valid(const YAML::Node& config);

/*!
 * Generate paths with max energy not more than max_energy_bound. Number of produced paths is greater or equal to the number of UAVs
 * @tparam F callable type for generating paths with the specified number of uavs. (int) -> mstsp_solver::final_solution_t
 * @param max_energy_bound maximum energy of one path in Joules
 * @param n_uavs Number of uavs. There will be no less paths than this number
 * @param f Function that generates the specified number of paths
 * @return Solution to the problem
 */

/*!
 * Read points from CSV file into vector of points
 */
std::vector<point_t> read_points_from_csv(const std::string& filename);

/*!
 * Write points into a CSV file with two columns
 */
void write_polygon_into_csv(const std::vector<point_heading_t<double>>& path, const std::string& filename);

/*!
 * Solve the algorithm for the specific number of UAV flights
 * @param n_uavs Number of UAVs -- exact number of paths to be generated
 * @param algorithm_config Algorithm configuration
 * @param polygon Polygon to solve for
 * @param energy_calculator Initialized energy calculator for the specific UAV
 * @param shortest_path_calculator Initialized shorted path calculator
 * @param logger Logger to log output
 */
//}



int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Error. Usage: coverage_mission_planner <algorithm_configuration>.yaml" << std::endl;
        return -1;
    }
    YAML::Node algorithm_config_node = YAML::LoadFile(argv[1]);
    if (!algorithm_config_is_valid(algorithm_config_node)) {
        std::cerr << "Algorithm config is not complete. Exiting..." << std::endl;
        return -1;
    }
    algorithm_config_t algorithm_config;
    try {
        algorithm_config = parse_algorithm_config(algorithm_config_node);
    } catch (const YAML::Exception &e) {
        std::cout << "Error while parsing YAML configuration file: " << e.what() << std::endl;
        return -1;
    }

    std::vector<std::vector<point_t>> no_fly_zones;
    for (const auto &s: algorithm_config.no_fly_zone_points_files) {
        auto no_fly_zone = read_points_from_csv(s);
        if (no_fly_zone.empty()) {
            std::cout << "Error: no fly zone file " << s << " is either empty or of a wrong format" << std::endl;
            return -1;
        }
        no_fly_zones.push_back(no_fly_zone);
    }

    std::vector<std::pair<polygon_t, double>> hr_no_fly_zones_data;
    for (const auto &s: algorithm_config.hr_no_fly_zone_files) {
        auto hr_no_fly_zone_points = read_points_from_csv(s.filename);
        if (hr_no_fly_zone_points.empty()) {
            std::cout << "Error: height restricted no fly zone file " << s.filename << " is empty or of a wrong format" << std::endl;
            return -1;
        }
        hr_no_fly_zones_data.push_back({hr_no_fly_zone_points, s.max_altitude});
    }

    // Create a logger to log everything directly into stdout
    auto shared_logger = std::make_shared<loggers::SimpleLogger>();

    std::vector<std::shared_ptr<PathCostCalculator>> cost_calculators;
    for (const auto& spec : algorithm_config.drones) {
        if (spec.optimization_type == "energy" && spec.energy_config.has_value()) {
            cost_calculators.push_back(std::make_shared<EnergyCalculator>(spec.energy_config.value(), shared_logger));
        } else if (spec.optimization_type == "time" && spec.time_config.has_value()) {
            cost_calculators.push_back(std::make_shared<TimeCalculator>(spec.time_config.value()));
        } else {
            shared_logger->log_err("Unknown or incomplete drone specification for optimization_type: " + spec.optimization_type);
            return -1;
        }
    }
    // Create one master polygon that contains ALL obstacles. This will be used for pathfinding between areas.
    // The fly-zone part is left empty, as the ShortestPathCalculator will ignore it anyway.
    MapPolygon master_obstacle_polygon;
    if (algorithm_config.points_in_lat_lon) {
        polygon_t empty_fly_zone;
        master_obstacle_polygon = MapPolygon(empty_fly_zone, no_fly_zones, algorithm_config.lat_lon_origin, hr_no_fly_zones_data);
    } else {
        std::vector<HeightRestrictedNoFlyZone> hr_nfz_structs;
        for(const auto& data : hr_no_fly_zones_data) {
            hr_nfz_structs.push_back({data.first, data.second});
        }
        polygon_t empty_fly_zone;
        master_obstacle_polygon = MapPolygon(empty_fly_zone, no_fly_zones, hr_nfz_structs);
    }
    ShortestPathCalculator shortest_path_calculator(master_obstacle_polygon, true, 0);

    // Now, create a vector of MapPolygon objects, one for each search area.
    // Each of these will contain its own fly zone boundary, but also ALL no-fly zones.
    // The trapezoidal decomposition will correctly handle only the NFZs inside the FZ.
    std::vector<MapPolygon> search_areas;
    for (const auto& fz_filename : algorithm_config.fly_zone_points_files) {
        auto fly_zone_points = read_points_from_csv(fz_filename);
        if (fly_zone_points.empty()) {
            std::cout << "Error: fly zone points file '" << fz_filename << "' is either empty or of wrong format" << std::endl;
            return -1;
        }

        MapPolygon area;
        if (algorithm_config.points_in_lat_lon) {
            area = MapPolygon(fly_zone_points, no_fly_zones, algorithm_config.lat_lon_origin, hr_no_fly_zones_data);
        } else {
            std::vector<HeightRestrictedNoFlyZone> hr_nfz_structs;
            for(const auto& data : hr_no_fly_zones_data) {
                hr_nfz_structs.push_back({data.first, data.second});
            }
            area = MapPolygon(fly_zone_points, no_fly_zones, hr_nfz_structs);
        }

        // Remove outer no-fly zones only for decomposition and sweeping for THIS specific area.
        // ShortestPathCalculator already loaded the original polygon with all zones for safe overflights.
        std::vector<polygon_t> internal_nfz;
        for (const auto& nfz : area.no_fly_zone_polygons) {
            if (!nfz.empty() && is_point_in_polygon(nfz[0], area.fly_zone_polygon_points)) {
                internal_nfz.push_back(nfz);
            }
        }
        area.no_fly_zone_polygons = internal_nfz;

        // Do the same for HR NFZs - keep only the internal ones for decomposition.
        std::vector<HeightRestrictedNoFlyZone> internal_hr_nfz;
        for (const auto& hr_nfz : area.height_restricted_no_fly_zone_polygons) {
             if (!hr_nfz.polygon.empty() && is_point_in_polygon(hr_nfz.polygon[0], area.fly_zone_polygon_points)) {
                internal_hr_nfz.push_back(hr_nfz);
            }
        }
        area.height_restricted_no_fly_zone_polygons = internal_hr_nfz;

        search_areas.push_back(area);
    }

    mstsp_solver::final_solution_t best_solution;
    try {
        best_solution = solve_for_uavs(algorithm_config.number_of_drones, algorithm_config, search_areas, cost_calculators, shortest_path_calculator, shared_logger);

    } catch (const polygon_decomposition_error &e) {
        std::cout << "Error while decomposing the polygon" << std::endl;
        return -1;
    } catch (const std::runtime_error &e) {
        std::cout << "Error while solving for polygons: " << e.what();
        return -1;
    }

    if (best_solution.paths.empty()) {
        std::cout << "Failed to find any valid solution." << std::endl;
        return 0;
    }

    // Check if path is within cost constraints
    for (int i = 0; i < algorithm_config.number_of_drones; i++) {
        double path_cost = cost_calculators.at(i)->calculate_path_cost(best_solution.paths.at(i));
        double cost_limit = algorithm_config.drones.at(i).max_single_path_cost;

        if (path_cost > cost_limit) {
            std::cout << "Found solution for drone " << i << " costs " << path_cost << " " << (algorithm_config.drones.at(0).optimization_type == "energy" ? "Joules" : "seconds") <<
                " which exceeds the limit of " << cost_limit << " " << (algorithm_config.drones.at(0).optimization_type == "energy" ? "Joules" : "seconds") <<
                ". Try to increase the number of drones,"<< std::endl;
            return 0;
        }
    }

    std::cout << "Writing output paths into files" << std::endl;
    auto best_paths = best_solution.paths;

    // If initial paths were read in lat_lon coordinates, write the output paths in the same way
    if (algorithm_config.points_in_lat_lon) {
        for (auto &path: best_paths) {
            for (auto &p: path) {
                auto lat_lon_p = meters_to_gps_coordinates({p.x, p.y}, algorithm_config.lat_lon_origin);
                p.x = lat_lon_p.first;
                p.y = lat_lon_p.second;
            }}
    }

    for (size_t i = 0; i < best_paths.size(); ++i) {
        write_polygon_into_csv(best_paths[i], "path_" + std::to_string(i) + ".csv");
    }

    return 0;
}



/* read_points_from_csv() //{ */

std::vector<point_t> read_points_from_csv(const std::string& filename)
{
  std::ifstream is(filename);

  if (!is.is_open())
  {
    std::cerr << "Error opening file " << filename << std::endl;
    return {};
  }

  std::vector<point_t> points;

  // Read the CSV file line by line
  std::string line;
  while (std::getline(is, line))
  {
    std::istringstream iss(line);
    std::string x_str, y_str;

    // Split the line into latitude and longitude
    if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ','))
    {
      // Add the pair to the vector
      points.emplace_back(std::stod(x_str), std::stod(y_str));
    } else
    {
      std::cerr << "Error parsing line: " << line << std::endl;
    }
  }

  is.close();
  // Make sure that the first point is always the same as the last one
  if (not points.empty() and points[0] != points[points.size() - 1])
  {
    points.push_back(points[0]);
  }
  return points;
}
//}

/* write_polygon_into_csv() //{ */

void write_polygon_into_csv(const std::vector<point_heading_t<double>>& path, const std::string& filename)
{
  std::ofstream of{filename};
  of << std::setprecision(10);
  for (const auto& p : path)
  {
    of << p.x << ", " << p.y << ", " << p.z << std::endl;
  }
  of.close();
}
//}

/* algorithm_config_is_valid() //{ */

bool algorithm_config_is_valid(const YAML::Node &config) {
    // Helper lambda for checking if a key exists
    auto check_key = [&](const YAML::Node& node, const std::string& key) {
        if (!node[key]) {
            std::cerr << "Error: Missing required key '" << key << "' in the config file." << std::endl;
            return false;
        }
        return true;
    };

    // --- Global mission parameters ---
    const std::vector<std::string> global_keys = {
        "number_of_rotations", "points_in_lat_lon",
        "sweeping_step", "decomposition_method", "min_sub_polygons_per_uav",
        "rotations_per_cell", "no_improvement_cycles_before_stop", "drones"
    };

    for (const auto& key : global_keys) {
        if (!check_key(config, key)) return false;
    }

    if (config["points_in_lat_lon"].as<bool>()) {
        if (!check_key(config, "latitude_origin") || !check_key(config, "longitude_origin")) {
            std::cerr << "Error: 'latitude_origin' and 'longitude_origin' are required when 'points_in_lat_lon' is true." << std::endl;
            return false;
        }
    }

    // --- Parameters for individual drones ---
    if (!config["drones"] || !config["drones"].IsSequence() || config["drones"].size() == 0) {
        std::cerr << "Error: 'drones' array is missing, not a sequence, or is empty in the config file." << std::endl;
        return false;
    }

    const std::string first_optimization_type = config["drones"][0]["optimization_type"].as<std::string>();

    for (const auto& drone_node : config["drones"]) {
        if (!check_key(drone_node, "optimization_type") || !check_key(drone_node, "max_single_path_cost")) {
            std::cerr << "Error: Each drone in 'drones' must have 'optimization_type' and 'max_single_path_cost'." << std::endl;
            return false;
        }

        const std::string current_optimization_type = drone_node["optimization_type"].as<std::string>();
        if (current_optimization_type != first_optimization_type) {
            std::cerr << "Error: All drones must have the same 'optimization_type'. Found '" << current_optimization_type
                      << "' which is different from the first drone's type '" << first_optimization_type << "'." << std::endl;
            return false;
        }


        if (current_optimization_type == "energy") {
            const std::vector<std::string> energy_keys = {
                "drone_mass", "drone_area", "average_acceleration", "propeller_radius",
                "number_of_propellers", "allowed_path_deviation", "battery_model", "best_speed_model"
            };
            for (const auto& key : energy_keys) {
                if (!check_key(drone_node, key)) {
                    std::cerr << "Error: Drone with 'energy' optimization is missing key '" << key << "'." << std::endl;
                    return false;
                }
            }
            if (!check_key(drone_node["battery_model"], "cell_capacity") || !check_key(drone_node["battery_model"], "number_of_cells") ||
                !check_key(drone_node["battery_model"], "d0") || !check_key(drone_node["battery_model"], "d1") ||
                !check_key(drone_node["battery_model"], "d2") || !check_key(drone_node["battery_model"], "d3")) {
                std::cerr << "Error: 'battery_model' node is missing one or more required parameters (cell_capacity, number_of_cells, d0-d3)." << std::endl;
                return false;
            }
            if (!check_key(drone_node["best_speed_model"], "c0") || !check_key(drone_node["best_speed_model"], "c1") ||
                !check_key(drone_node["best_speed_model"], "c2")) {
                std::cerr << "Error: 'best_speed_model' node is missing one or more required parameters (c0-c2)." << std::endl;
                return false;
            }

            // Check global physical parameters required for the energy model
            if (!check_key(config, "air_density") || !check_key(config, "earth_gravity") || !check_key(config, "propeller_efficiency")) {
                 return false;
            }

        } else if (current_optimization_type == "time") {
            const std::vector<std::string> time_keys = {
                "max_horizontal_speed", "max_vertical_speed", "horizontal_acceleration",
                "vertical_acceleration", "allowed_path_deviation"
            };
            for (const auto& key : time_keys) {
                if (!check_key(drone_node, key)) {
                    std::cerr << "Error: Drone with 'time' optimization is missing key '" << key << "'." << std::endl;
                    return false;
                }
            }
        } else {
            std::cerr << "Error: Unknown 'optimization_type': " << current_optimization_type << ". Use 'energy' or 'time'." << std::endl;
            return false;
        }
    }

    return true;
}
//}

/* parse_algorithm_config() //{ */

algorithm_config_t parse_algorithm_config(const YAML::Node& config) {
    algorithm_config_t algorithm_config;

    // Load global physical parameters
    const double air_density = config["air_density"] ? config["air_density"].as<double>() : 1.225;
    const double earth_gravity = config["earth_gravity"] ? config["earth_gravity"].as<double>() : 9.81;
    const double propeller_efficiency = config["propeller_efficiency"] ? config["propeller_efficiency"].as<double>() : 0.6;

    algorithm_config.number_of_drones = config["drones"].size();
    for (const auto& drone_node : config["drones"]) {
        drone_spec_t spec;
        spec.optimization_type = drone_node["optimization_type"].as<std::string>();
        spec.max_single_path_cost = drone_node["max_single_path_cost"].as<double>();

        if (spec.optimization_type == "energy") {
            energy_calculator_config_t energy_conf;
            energy_conf.allowed_path_deviation = drone_node["allowed_path_deviation"].as<double>();
            energy_conf.drone_mass = drone_node["drone_mass"].as<double>();
            energy_conf.drone_area = drone_node["drone_area"].as<double>();
            energy_conf.average_acceleration = drone_node["average_acceleration"].as<double>();
            energy_conf.propeller_radius = drone_node["propeller_radius"].as<double>();
            energy_conf.number_of_propellers = drone_node["number_of_propellers"].as<int>();
            auto battery_model_config = drone_node["battery_model"];
            
            energy_conf.air_density = air_density;
            energy_conf.earth_gravity = earth_gravity;
            energy_conf.propeller_efficiency = propeller_efficiency;
            energy_conf.battery_model = {battery_model_config["cell_capacity"].as<double>(), battery_model_config["number_of_cells"].as<int>(), battery_model_config["d0"].as<double>(), battery_model_config["d1"].as<double>(), battery_model_config["d2"].as<double>(), battery_model_config["d3"].as<double>()};
            auto best_speed_model_config = drone_node["best_speed_model"];
            energy_conf.best_speed_model = {best_speed_model_config["c0"].as<double>(), best_speed_model_config["c1"].as<double>(), best_speed_model_config["c2"].as<double>()};
            spec.energy_config = energy_conf;
        } else if (spec.optimization_type == "time") {
            time_calculator_config_t time_conf;
            time_conf.allowed_path_deviation = drone_node["allowed_path_deviation"].as<double>();
            time_conf.max_speed = drone_node["max_speed"].as<double>();
            time_conf.max_acceleration = drone_node["max_acceleration"].as<double>();
            spec.time_config = time_conf;
        }
        algorithm_config.drones.push_back(spec);
    }

  // Common parameters
  algorithm_config.number_of_rotations = config["number_of_rotations"].as<int>();

  algorithm_config.points_in_lat_lon = config["points_in_lat_lon"].as<bool>();
  if (algorithm_config.points_in_lat_lon)
  {
    algorithm_config.lat_lon_origin = {config["latitude_origin"].as<double>(), config["longitude_origin"].as<double>()};
  }

  if (config["no_fly_zones_filenames"])
  {
    for (const auto& node : config["no_fly_zones_filenames"])
    {
      algorithm_config.no_fly_zone_points_files.emplace_back(node.as<std::string>());
    };
  }

  if (config["height_restricted_no_fly_zones"]) {
        for (const auto &node: config["height_restricted_no_fly_zones"]) {
            algorithm_config.hr_no_fly_zone_files.push_back({
                                                                    node["filename"].as<std::string>(),
                                                                    node["max_altitude"].as<double>()
                                                            });
        };
    }

    if (config["fly_zone_filenames"]) {
        algorithm_config.fly_zone_points_files = config["fly_zone_filenames"].as<std::vector<std::string>>();
    }

  algorithm_config.sweeping_step = config["sweeping_step"].as<int>();
  algorithm_config.decomposition_type = static_cast<decomposition_type_t>(config["decomposition_method"].as<int>());
  algorithm_config.min_sub_polygons_per_uav = config["min_sub_polygons_per_uav"].as<int>();

  algorithm_config.start_pos = {config["start_x"].as<double>(), config["start_y"].as<double>()};
  algorithm_config.rotations_per_cell = config["rotations_per_cell"].as<int>();
  algorithm_config.no_improvement_cycles_before_stop = config["no_improvement_cycles_before_stop"].as<int>();

  return algorithm_config;
}
//}

/* solve_for_uavs() //{ */

[[maybe_unused]] mstsp_solver::final_solution_t solve_for_uavs(int n_uavs, const algorithm_config_t& algorithm_config,
                                                        const std::vector<MapPolygon> &search_areas,
                                                        const std::vector<std::shared_ptr<PathCostCalculator>>& cost_calculators,
                                                        const ShortestPathCalculator& shortest_path_calculator,
                                                        std::shared_ptr<loggers::SimpleLogger>& logger)
{
  logger->log_info("Solving for " + std::to_string(n_uavs) + " UAVs.");
  if (search_areas.empty()) {
    logger->log_err("solve_for_uavs called with no search areas.");
    return {};
  }

  // Find the largest search area to use as a representative for finding best decomposition angles
  const MapPolygon& representative_polygon = *std::max_element(search_areas.begin(), search_areas.end(),
      [](const auto& a, const auto& b){ return a.area() < b.area(); });

  auto best_initial_rotations = n_best_init_decomp_angles(representative_polygon, algorithm_config.number_of_rotations,
                                                          algorithm_config.decomposition_type);

  std::cout << "Calculated best rotations: " << std::endl;
  for (const auto& rot : best_initial_rotations)
  {
    std::cout << rot << std::endl;
  }

  // Run algorithm for each rotation and save the best result
  double best_solution_cost = std::numeric_limits<double>::max();
  mstsp_solver::final_solution_t best_solution;
  for (const auto& rotation : best_initial_rotations)
  {
    std::vector<MapPolygon> all_decomposed_cells;

    // Decompose each search area using the current rotation and collect all resulting cells
    for (const auto& area : search_areas) {
        MapPolygon rotated_area = area.rotated(rotation);
        auto decomposed_cells = trapezoidal_decomposition(rotated_area, static_cast<decomposition_type_t>(algorithm_config.decomposition_type));
        all_decomposed_cells.insert(all_decomposed_cells.end(), decomposed_cells.begin(), decomposed_cells.end());
    }

    std::cout << "All areas decomposed for rotation " << rotation << ". Total cells: " << all_decomposed_cells.size() << std::endl;
    for (const auto &p: all_decomposed_cells) {
        std::cout << "  - Decomposed sub polygon area: " << p.area() << std::endl;
    }

    // Divide large polygons into smaller ones to meet the constraint on the lowest number of sub polygons
    std::cout << "Dividing large polygons into smaller ones" << std::endl;
    std::vector<MapPolygon> polygons_divided;
    try
    {
      polygons_divided = split_into_number(all_decomposed_cells, static_cast<size_t>(n_uavs) * algorithm_config.min_sub_polygons_per_uav);
    }
    catch (std::runtime_error& e)
    {
      std::cout << "ERROR while dividing polygon: " << e.what() << std::endl;
      return best_solution;
    }

    for (auto& p : polygons_divided)
    {
      p = p.rotated(-rotation);
    }
    std::cout << "Divided large polygons into smaller ones" << std::endl;

    // Create the configuration for MSTSP solver
    auto starting_point = algorithm_config.points_in_lat_lon ? gps_coordinates_to_meters(algorithm_config.start_pos, algorithm_config.lat_lon_origin)
                                                             : algorithm_config.start_pos;
    mstsp_solver::SolverConfig solver_config{algorithm_config.rotations_per_cell,
                                             algorithm_config.sweeping_step,
                                              starting_point,
                                             static_cast<size_t>(n_uavs),
                                             algorithm_config.sweeping_alt,
                                              0,
                                              algorithm_config.no_improvement_cycles_before_stop};
    solver_config.wall_distance = algorithm_config.sweeping_step / 2;
    mstsp_solver::MstspSolver solver(solver_config, polygons_divided, cost_calculators, shortest_path_calculator);
    solver.set_logger(logger);

    auto solver_res = solver.solve();

    // Change the best solution if the current one is better
    if (solver_res.max_path_energy < best_solution_cost)
    {
        best_solution_cost = solver_res.max_path_energy;
        best_solution = solver_res;
        std::cout << "Best solution rotation: " << rotation / M_PI * 180 << std::endl;
    }
  }
  return best_solution;
}

/* decompose_polygon() //{ */

[[maybe_unused]] std::vector<MapPolygon> decompose_polygon(int n_uavs, const algorithm_config_t& algorithm_config, MapPolygon polygon)
{
  auto init_polygon = polygon;

  auto best_initial_rotations = n_best_init_decomp_angles(polygon, algorithm_config.number_of_rotations, algorithm_config.decomposition_type);

  std::cout << "Calculated best rotations: " << std::endl;
  for (const auto& rot : best_initial_rotations)
  {
    std::cout << rot << std::endl;
  }

  std::vector<MapPolygon> polygons_decomposed;

  // Run algorithm for each rotation and save the best result
  for (const auto& rotation : best_initial_rotations)
  {
    // Decompose polygon using initial rotation
    polygon = init_polygon.rotated(rotation);
    polygons_decomposed = trapezoidal_decomposition(polygon, static_cast<decomposition_type_t>(algorithm_config.decomposition_type));

    std::cout << "Polygon decomposed. Decomposed polygons: " << std::endl;
    for (const auto& p : polygons_decomposed)
    {
      std::cout << "Decomposed sub polygon area: " << p.area() << std::endl;
    }

    // Divide large polygons into smaller ones to meet the constraint on the lowest number of sub polygons
    std::cout << "Dividing large polygons into smaller ones" << std::endl;
    std::vector<MapPolygon> polygons_divided;
    try
    {
      polygons_divided = split_into_number(polygons_decomposed, static_cast<size_t>(n_uavs) * algorithm_config.min_sub_polygons_per_uav);
    }
    catch (std::runtime_error& e)
    {
      std::cout << "ERROR while dividing polygon: " << e.what() << std::endl;
    }

    for (auto& p : polygons_divided)
    {
      p = p.rotated(-rotation);
    }
    polygons_decomposed = polygons_divided;
    std::cout << "Divided large polygons into smaller ones" << std::endl;

    //Print number of polygons
    std::cout << "Number of polygons: " << polygons_decomposed.size() << std::endl;
  }
  return polygons_decomposed;
}
//}
