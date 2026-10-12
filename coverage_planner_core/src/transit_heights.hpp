#ifndef COVERAGE_PLANNER_CORE_TRANSIT_HEIGHTS_HPP
#define COVERAGE_PLANNER_CORE_TRANSIT_HEIGHTS_HPP

// Transit-height deconfliction, ported from iroc_fleet_manager's CoveragePlanner plugin.
//
// Every drone path is a sequence of points; consecutive points above the sweep altitude form a "transit path group"
// (TPG). TPGs of different drones that come closer than their minimum horizontal distance get different altitudes,
// assigned greedily (graph colouring by overlap degree), and a TPG that crosses another drone's sweep lanes is lifted
// above the sweep altitude.

#include <memory>
#include <vector>

#include <EnergyAwareMCPP/custom_types.hpp>

namespace coverage_planner_core::detail
{

using path_t = std::vector<point_heading_t<double>>;

struct TransitPath
{
  double x1, y1, x2, y2;
};

class TransitPathGroup
{
public:
  TransitPathGroup(int drone_idx, double min_horizontal_distance, double min_vertical_distance)
      : drone_idx(drone_idx), min_horizontal_distance(min_horizontal_distance), min_vertical_distance(min_vertical_distance)
  {
  }

  const std::vector<TransitPath>& get() const { return paths_; }

  // height == transit_height: the group stays on the default transit level (it may still be lifted);
  // otherwise it already flies over a height-restricted zone and must stay above that.
  void setHeight(double height, double sweeping_height, double transit_height)
  {
    drone_height = (height == transit_height) ? sweeping_height : height + min_vertical_distance;
  }

  void writeHeights(double height)
  {
    for (double* z : z_ptrs_) {
      *z = height;
    }
  }

  void addTransitPath(const TransitPath& tp, double* z1, double* z2);

  int drone_idx;
  double min_horizontal_distance;
  double min_vertical_distance;
  double drone_height = 0.0;

private:
  std::vector<TransitPath> paths_;
  std::vector<double*> z_ptrs_;
};

/*!
 * Assign altitudes to the transit parts of @p paths in place.
 * @param paths one path per drone, in metres; z of sweep points == sweeping_height, z of transit points > sweeping_height
 */
void resolve_transit_heights(std::vector<path_t>& paths, double sweeping_height, double transit_height,
                             const std::vector<double>& min_horizontal_distances, const std::vector<double>& min_vertical_distances);

/// Minimum-cost assignment; returns for each row the assigned column. Square matrix expected.
std::vector<int> hungarian(const std::vector<std::vector<double>>& matrix);

/// True when two segments cross.
bool segments_intersect(const TransitPath& a, const TransitPath& b);

}  // namespace coverage_planner_core::detail

#endif  // COVERAGE_PLANNER_CORE_TRANSIT_HEIGHTS_HPP
