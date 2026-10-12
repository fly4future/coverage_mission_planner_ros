#include "mstsp_solver/TargetSet.h"
#include "algorithms.hpp"
#include <iostream>
#include <algorithm>
#include <utility>
#include "PathCostCalculator.hpp"

namespace mstsp_solver
{
  /* TargetSet Constructor //{ */

  TargetSet::TargetSet(size_t index, const MapPolygon& polygon, double sweeping_step, double wall_distance, const std::vector<std::shared_ptr<PathCostCalculator>>* cost_calculators,
                       const std::vector<double>& rotation_angles)
      : index(index), polygon(polygon), cost_calculators(cost_calculators), sweeping_step(sweeping_step), m_wall_distance{wall_distance}
  {
    set_rotation_angles(rotation_angles); // Note: sweep_alt is not available here, assuming 0. This might need adjustment.
  }
  //}

  /* TargetSet Constructor //{ */
  
  TargetSet::TargetSet(size_t index, const MapPolygon& polygon, double sweeping_step, double wall_distance, const std::vector<std::shared_ptr<PathCostCalculator>>* cost_calculators,
                       size_t number_of_edges_rotations)
      : index(index), polygon(polygon), cost_calculators(cost_calculators), sweeping_step(sweeping_step), m_wall_distance{wall_distance}
  {

    auto thin_coverage = thin_polygon_coverage(polygon, sweeping_step, 4);
    // If no thin coverage path is generated because the polygon is not thin enough, perform normal sweeping procedure
    if (thin_coverage.empty())
    {
      set_rotation_angles(polygon.get_n_longest_edges_rotation_angles(number_of_edges_rotations));
    } else
    {
      Target target{true, 0.0, {}, thin_coverage[0], thin_coverage.back(), index, targets.size()};
      target.drones_sweep_costs.resize(cost_calculators->size());
      for (size_t uav_idx = 0; uav_idx < cost_calculators->size(); ++uav_idx) {
          // Assuming thin_coverage is 2D, we need to create a 3D path for cost calculation
          std::vector<point_heading_t<double>> path_3d;
          for(const auto& p : thin_coverage) path_3d.emplace_back(p);
          target.drones_sweep_costs[uav_idx] = (*cost_calculators)[uav_idx]->calculate_path_cost(path_3d);
      }
      targets.push_back(target);
    }
  }
  //}

  /* TargetSet::add_one_rotation_angle() //{ */
  
  void TargetSet::add_one_rotation_angle(double angle, bool up)
  {
    auto sweeping_path = sweeping(polygon, angle, sweeping_step, m_wall_distance, up);
    // If sweeping failed (e.g. because of the polygon splitting with such a rotation angle)
    if (sweeping_path.empty()) { return; }

    std::vector<point_heading_t<double>> path_with_z;
    path_with_z.reserve(sweeping_path.size());
    for(const auto& p : sweeping_path) {
        point_heading_t<double> p_3d(p);
        path_with_z.push_back(p_3d);
    }
    
    Target target{up, angle, {}, sweeping_path[0], sweeping_path.back(), index, targets.size()};
    target.drones_sweep_costs.resize(cost_calculators->size());
    for (size_t uav_idx = 0; uav_idx < cost_calculators->size(); ++uav_idx) {
        target.drones_sweep_costs[uav_idx] = (*cost_calculators)[uav_idx]->calculate_path_cost(path_with_z);
    }
    
    targets.push_back(target);
  }
  //}

  /* TargetSet::set_rotation_angles() //{ */
  
  void TargetSet::set_rotation_angles(const std::vector<double>& angles)
  {
    targets.clear();
    for (auto angle : angles)
    {
      for (int i = 0; i < 2; i++)
      {
        add_one_rotation_angle(angle, static_cast<bool>(i));
      }
    }
    if (targets.empty())
    {
      // If not sweeping angle produced a valid sweeping pattern
      // Try to add the sweeping with no angle. This should work for any polygon after boustrophedon decomposition
      add_one_rotation_angle(0, true);
      add_one_rotation_angle(0, false);
    }
  }
  //}
}  // namespace mstsp_solver
