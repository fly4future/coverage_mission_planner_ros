#include "EnergyAwareMCPP/MapPolygon.hpp"
#include "EnergyAwareMCPP/EnergyCalculator.h"
#include "EnergyAwareMCPP/algorithms.hpp"
#include "EnergyAwareMCPP/ShortestPathCalculator.hpp"
#include "EnergyAwareMCPP/mstsp_solver/SolverConfig.h"
#include "EnergyAwareMCPP/mstsp_solver/MstspSolver.h"
#include <iostream>
#include "EnergyAwareMCPP/SimpleLogger.h"
#include "EnergyAwareMCPP/utils.hpp"
#include <EnergyAwareMCPP/coverage_planner.hpp>

mstsp_solver::final_solution_t solve_for_uavs(int n_uavs, const algorithm_config_t& algorithm_config,
                                              const std::vector<MapPolygon> &search_areas,
                                              const EnergyCalculator& energy_calculator,
                                              const ShortestPathCalculator& shortest_path_calculator,
                                              std::shared_ptr<loggers::SimpleLogger>& logger)
{
  std::cout << "[DEBUG solve_for_uavs] n_uavs: " << n_uavs << std::endl;
  std::cout << "[DEBUG solve_for_uavs] algorithm_config: "
            << "number_of_rotations=" << algorithm_config.number_of_rotations
            << ", decomposition_type=" << (int)algorithm_config.decomposition_type
            << ", sweeping_step=" << algorithm_config.sweeping_step
            << ", sweeping_alt=" << algorithm_config.sweeping_alt
            << ", min_sub_polygons_per_uav=" << algorithm_config.min_sub_polygons_per_uav
            << ", start_pos=(" << algorithm_config.start_pos.first << ", " << algorithm_config.start_pos.second << ")"
            << ", max_single_path_energy=" << algorithm_config.max_single_path_energy << std::endl;
  std::cout << "[DEBUG solve_for_uavs] search_areas size: " << search_areas.size() << std::endl;
  for (size_t i = 0; i < search_areas.size(); ++i) {
    std::cout << "[DEBUG solve_for_uavs] search_area[" << i << "] area: " << search_areas[i].area() << std::endl;
  }

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
    mstsp_solver::MstspSolver solver(solver_config, polygons_divided, energy_calculator, shortest_path_calculator);
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
