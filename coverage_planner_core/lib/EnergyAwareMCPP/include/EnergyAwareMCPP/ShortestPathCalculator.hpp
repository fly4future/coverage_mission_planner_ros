#ifndef THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
#define THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP

#include <map>
#include <vector>
#include "MapPolygon.hpp"
#include <unordered_map>

struct shortest_path_calculation_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

/*!
 * Struct to make hashing of pair of points possible
 */
struct point_pair_hash {
    size_t operator()(const std::pair<point_t, point_t> &p) const {
        auto hash = std::hash<double>{};
        return hash(p.first.first) + hash(p.first.second) + hash(p.second.first) + hash(p.second.second);
    };
};

/*!
 * Class for calculation of the shortest path between points inside a polygon
 */
class ShortestPathCalculator {
private:
    std::vector<HeightRestrictedNoFlyZone> m_hr_no_fly_zone_polygons;
    std::vector<point_t> m_polygon_points;
    mutable std::unordered_map<std::pair<point_t, point_t>, std::pair<std::vector<point_t>, double>, point_pair_hash> paths_cache;

    const MapPolygon& m_base_map_polygon; // Reference to the original MapPolygon
    std::vector<segment_t> m_polygon_segments; // Base polygon segments (fly zone + regular no-fly zones)

    double sweeping_height;
    double transit_height;

    /*!
     * Check if a direct line segment between two points intersects any of the base polygon segments.
     * This does NOT consider height-restricted no-fly zones.
     * @param p1 First point
     * @param p2 Second point
     * @return True if the segment (p1, p2) does not intersect any base polygon segments, false otherwise.
     */
    bool point_can_see_point(point_t p1, point_t p2) const;

    bool point_can_see_point(point_t p1, point_t p2, const std::vector<polygon_t>& additional_obstacle_polygons) const;

    /*!
     * Find the shortest path between two points by going around obstacles using the visibility graph.
     * @param p1 source point
     * @param p2 destination point
     * @param additional_obstacle_polygons Additional polygons to consider as obstacles for visibility checks.
     *                                     These are not part of the pre-calculated visibility graph, but are checked for direct line-of-sight.
     * @return path between points
     */
    std::vector<point_t> find_path_around_obstacles(point_t p1, point_t p2, const std::vector<polygon_t>& additional_obstacle_polygons = {}) const;


public:
    /*!
     * Main and the only constructor of the calculator.
     * @param polygon polygon, bounds of which will define the shortest path
     */
    explicit ShortestPathCalculator(const MapPolygon &polygon, bool ignore_fly_zone = false, double sweeping_height = 0);

    ShortestPathCalculator() = delete;

    /*!
     * Get the exact Euclidean shortest path between two points inside of the polygon
     * @param p1 source point
     * @param p2 destination point
     * @return path between point including the start and end node
     */
    std::pair<std::vector<point_t>, double> shortest_path_between_points(point_t p1, point_t p2) const;

    // Helper function to check if a path segment intersects any of the given polygons
    friend bool path_segment_intersects_polygons(const segment_t& path_segment, const std::vector<polygon_t>& polygons);
};

/*!
 * Helper function to check if a path segment intersects any of the given polygons.
 * @param path_segment The segment to check for intersection.
 * @param polygons A vector of polygons to check against.
 * @return True if the path segment intersects any of the provided polygons, false otherwise.
 */
bool path_segment_intersects_polygons(const segment_t& path_segment, const std::vector<polygon_t>& polygons);


#endif //THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP