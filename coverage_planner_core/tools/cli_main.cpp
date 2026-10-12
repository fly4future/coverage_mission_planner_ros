// Command-line front end of the EnergyAwareMCPP library (formerly the library's own main()).
// Usage: coverage_planner_cli <algorithm_configuration>.yaml   -> writes path_<i>.csv into the working directory

#include <coverage_planner.hpp>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Error. Usage: coverage_planner_cli <algorithm_configuration>.yaml" << std::endl;
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



