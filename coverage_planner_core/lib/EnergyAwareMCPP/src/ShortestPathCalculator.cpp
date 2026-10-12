#include "ShortestPathCalculator.hpp"
#include <algorithm>
#include "utils.hpp"
#include <cmath>

namespace
{
    const double EPS = 1e-5;


    /*!
     * Calculate the sets of polygon point pairs for which there is no direct path in a straight line.
     *
     * The set will include the pairs that can "see" each other through the center of the polygon for no-fly-zones
     * e.g. all possible pairs for a convex polygon except of neighbouring ones
     *
     * For a fly-zone, the set will include points that can "see" each other from outside of the polygon
     * e.g. if the polygon is concave, there is always a pair of points that can "see" each other without
     * any segments between them, but the direct path goes outside of the fly-zone
     *
     * @param polygon Polygon as a set of vertices
     * @param fly_zone Whether the polygon should be treated as a fly-zone
     * @return Set of polygon vertices between which there is no direct path not leaving the fly-zone
     */
    std::set<segment_t> calculate_no_direct_path_pairs(const std::vector<point_t> &polygon_points, bool is_fly_zone = true) {
        std::set<segment_t> no_direct_view;
        if (polygon_points.size() < 3) {
            return no_direct_view;
        }

        size_t n_points = polygon_points.size();
        if (polygon_points.front() == polygon_points.back()) {
            n_points--;
        }

        for (size_t i = 0; i < n_points; ++i) {
            for (size_t j = i + 2; j < n_points; ++j) {
                // Skip the edge connecting the first and the last node
                if (i == 0 && j == n_points - 1) continue;

                point_t midpoint = {(polygon_points[i].first + polygon_points[j].first) / 2.0,
                                    (polygon_points[i].second + polygon_points[j].second) / 2.0};
                bool inside = is_point_in_polygon(midpoint, polygon_points);
                
                if (is_fly_zone && !inside) {
                    no_direct_view.insert({polygon_points[i], polygon_points[j]});
                } else if (!is_fly_zone && inside) {
                    no_direct_view.insert({polygon_points[i], polygon_points[j]});
                }
            }
        }
        return no_direct_view;
    }

    /*!
     * Calculate the segments, between which there is no direct path definitely.
     * This version considers the base fly zone and regular no-fly zones.
     * Height-restricted no-fly zones are *not* included here.
     * @param map_polygon MapPolygon for which segments will be found
     * @param ignore_fly_zone If true, only no-fly zones are considered. If false, fly-zone and no-fly zones are considered.
     * @return Set of segment pairs that cannot be directly traversed.
     */
    std::set<segment_t> get_base_no_direct_path_pairs(const MapPolygon &map_polygon, bool ignore_fly_zone) {
        std::set<segment_t> no_direct_view;
        if (!ignore_fly_zone) {
            no_direct_view = calculate_no_direct_path_pairs(map_polygon.fly_zone_polygon_points, true);
        }
        for (const auto &p: map_polygon.no_fly_zone_polygons) {
            auto no_fly_zone_no_view = calculate_no_direct_path_pairs(p, false);
            no_direct_view.insert(no_fly_zone_no_view.begin(), no_fly_zone_no_view.end());
        }
        return no_direct_view;
    }
}

ShortestPathCalculator::ShortestPathCalculator(const MapPolygon &polygon, bool ignore_fly_zone, double sweeping_height) : m_base_map_polygon(polygon), sweeping_height(sweeping_height) {
  m_hr_no_fly_zone_polygons = polygon.height_restricted_no_fly_zone_polygons;
  if (ignore_fly_zone) {
    std::set<point_t> points_tmp;
    for (const auto &nfz : polygon.no_fly_zone_polygons) {
      std::copy(nfz.begin(), nfz.end(), std::inserter(points_tmp, points_tmp.begin()));
      for (size_t i = 0; i + 1 < nfz.size(); ++i) {
        m_polygon_segments.emplace_back(nfz[i], nfz[i + 1]); // These are regular no-fly zones, so they are part of the base segments
      }
    }
    m_polygon_points = std::vector<point_t>{points_tmp.begin(), points_tmp.end()};
  } else {
    m_polygon_points = std::vector<point_t>{m_base_map_polygon.get_base_points().begin(), m_base_map_polygon.get_base_points().end()};
    m_polygon_segments = m_base_map_polygon.get_base_segments();
  }

  transit_height = sweeping_height + 1.0;
}


// This algorithm uses a dynamic Dijkstra's algorithm to find the path.
// This correctly incorporates additional obstacles (both their boundaries for avoidance and their vertices for tight detours).
std::vector<point_t> ShortestPathCalculator::find_path_around_obstacles(point_t p1, point_t p2, const std::vector<polygon_t>& additional_obstacle_polygons) const {

    // Check direct visibility with all obstacles (base + additional)
    if (point_can_see_point(p1, p2, additional_obstacle_polygons)) {
        return {p1, p2}; // Direct path is possible
    }

    // Prepare all graph nodes: base points + vertices of additional obstacles + start and end points
    std::vector<point_t> all_nodes = m_polygon_points;
    for (const auto& poly : additional_obstacle_polygons) {
        for (const auto& pt : poly) {
            all_nodes.push_back(pt);
        }
    }
    all_nodes.push_back(p1);
    all_nodes.push_back(p2);
    
    size_t start_idx = all_nodes.size() - 2;
    size_t end_idx = all_nodes.size() - 1;

    // To avoid connecting through the interior (diagonals) of forbidden zones, 
    // we get all forbidden direct connections.
    std::set<segment_t> all_no_direct_paths = get_base_no_direct_path_pairs(m_base_map_polygon, false);
    for (const auto& poly : additional_obstacle_polygons) {
        auto ndp = calculate_no_direct_path_pairs(poly, false); // Additional obstacles are "no-fly zones"
        all_no_direct_paths.insert(ndp.begin(), ndp.end());
    }

    // Dijkstra's algorithm on a graph constructed on-the-fly
    size_t N = all_nodes.size();
    std::vector<double> dist(N, std::numeric_limits<double>::max());
    std::vector<size_t> parent(N, SIZE_MAX);
    std::vector<bool> visited(N, false);

    dist[start_idx] = 0.0;

    for (size_t i = 0; i < N; ++i) {
        // Select an unvisited node with the smallest distance
        double min_dist = std::numeric_limits<double>::max();
        size_t u = SIZE_MAX;
        for (size_t j = 0; j < N; ++j) {
            if (!visited[j] && dist[j] < min_dist) {
                min_dist = dist[j];
                u = j;
            }
        }

        // We are at the destination or the rest of the graph is unreachable
        if (u == SIZE_MAX || u == end_idx) break;
        visited[u] = true;

        // Update the distances of neighbors
        for (size_t v = 0; v < N; ++v) {
            if (!visited[v]) {
                double weight = distance_between_points(all_nodes[u], all_nodes[v]);
                // Optimization: perform the expensive visibility check only if the edge can improve the distance
                if (dist[u] + weight < dist[v]) {
                    segment_t seg{all_nodes[u], all_nodes[v]};
                    segment_t rev_seg{all_nodes[v], all_nodes[u]};
                    
                    // Check for disallowed lines from the interior of polygons and for visibility across edges
                    if (!all_no_direct_paths.count(seg) && !all_no_direct_paths.count(rev_seg)) {
                        if (point_can_see_point(all_nodes[u], all_nodes[v], additional_obstacle_polygons)) {
                            dist[v] = dist[u] + weight;
                            parent[v] = u;
                        }
                    }
                }
            }
        }
    }

    // If a path could not be found (fallback)
    if (parent[end_idx] == SIZE_MAX) {
        return {p1, p2};
    }

    // Reconstruct the shortest found path
    std::vector<point_t> path;
    size_t curr = end_idx;
    while (curr != SIZE_MAX) {
        path.push_back(all_nodes[curr]);
        curr = parent[curr];
    }
    std::reverse(path.begin(), path.end());

    // Clean up any immediately adjacent duplicate points
    std::vector<point_t> clean_path;
    for (const auto& pt : path) {
        if (clean_path.empty() || distance_between_points(clean_path.back(), pt) > 1e-5) {
            clean_path.push_back(pt);
        }
    }

    return clean_path;
}

// Helper function to check if a path segment intersects any of the given polygons
bool path_segment_intersects_polygons(const segment_t& path_segment, const std::vector<polygon_t>& polygons) {
    for (const auto& poly : polygons) {
        for (size_t i = 0; i + 1 < poly.size(); ++i) {
            segment_t poly_segment = {poly[i], poly[i+1]};
            if (segments_intersect(path_segment, poly_segment)) {
                return true;
            }
        }
    }
    return false;
}

std::pair<std::vector<point_t>, double> ShortestPathCalculator::shortest_path_between_points(point_t p1, point_t p2) const {
    // If path is saved to cache - return it from there
    auto path_in_cache = paths_cache.find({p1, p2});
    if (path_in_cache != paths_cache.end()) {
        return path_in_cache->second;
    }
    
    struct Candidate {
        double total_cost;
        std::vector<point_t> path;
        double transit_altitude;
    };
    std::vector<Candidate> candidate_paths;
    std::vector<polygon_t> current_hard_hr_nfzs; // HR-NFZs that are currently treated as hard obstacles

    // Loop to iteratively add HR-NFZs as hard obstacles
    while (true) {
        // Calculate the shortest path around the current set of hard obstacles
        // This uses the base visibility graph (m_polygon_points, m_floyd_warshall_d)
        // but checks visibility from p1/p2 to polygon points against ALL obstacles.
        std::vector<point_t> current_path = find_path_around_obstacles(p1, p2, current_hard_hr_nfzs);

        // Check for intersections with *remaining* HR-NFZs (those not yet added to current_hard_hr_nfzs)
        double max_intersected_hr_nfz_altitude = 0.0;
        const HeightRestrictedNoFlyZone* highest_hr_nfz_intersected = nullptr;
        
        std::vector<segment_t> path_segments;
        for (size_t i = 0; i + 1 < current_path.size(); ++i) {
            path_segments.push_back({current_path[i], current_path[i+1]});
        }

        bool intersects_any_remaining_hr_nfz = false;
        for (const auto& hr_nfz : m_hr_no_fly_zone_polygons) {
            // Check if this HR-NFZ is already a hard obstacle
            bool already_hard = false;
            for (const auto& hard_poly : current_hard_hr_nfzs) {
                if (hr_nfz.polygon == hard_poly) { // Simple comparison, might need more robust check
                    already_hard = true;
                    break;
                }
            }
            if (already_hard) continue; // Skip if already a hard obstacle

            // Check if current path intersects this HR-NFZ
            bool intersects_this_hr_nfz = false;
            for (const auto& path_seg : path_segments) {
                if (path_segment_intersects_polygons(path_seg, {hr_nfz.polygon})) {
                    intersects_this_hr_nfz = true;
                    break;
                }
            }

            if (intersects_this_hr_nfz) {
                intersects_any_remaining_hr_nfz = true;
                if (hr_nfz.max_altitude > max_intersected_hr_nfz_altitude) {
                    max_intersected_hr_nfz_altitude = hr_nfz.max_altitude;
                    highest_hr_nfz_intersected = &hr_nfz;
                }
            }
        }

        // Calculate path cost (distance + penalty)
        double path_distance = 0.0;
        for (size_t i = 0; i + 1 < current_path.size(); ++i) {
            path_distance += distance_between_points(current_path[i], current_path[i+1]);
        }
        double total_cost = path_distance;
        double current_transit_alt = transit_height;
        if (intersects_any_remaining_hr_nfz) {
            total_cost += 2 * std::max(max_intersected_hr_nfz_altitude - sweeping_height, 0.0); // Add penalty for overflying the highest intersected HR-NFZ
            current_transit_alt = std::max(transit_height, max_intersected_hr_nfz_altitude);
        }
        candidate_paths.push_back({total_cost, current_path, current_transit_alt});

        if (!intersects_any_remaining_hr_nfz) {
            break; // No more HR-NFZs intersected, we are done
        } else {
            // Add the highest intersected HR-NFZ to hard obstacles for the next iteration
            current_hard_hr_nfzs.push_back(highest_hr_nfz_intersected->polygon);
        }
    }

    // Find the path with the minimum total cost among all candidates
    double min_total_cost = std::numeric_limits<double>::max();
    std::pair<std::vector<point_t>, double> best_path;
    for (const auto& candidate : candidate_paths) {
        if (candidate.total_cost < min_total_cost) {
            min_total_cost = candidate.total_cost;
            best_path = {candidate.path, candidate.transit_altitude};
        }
    }

    paths_cache[{p1, p2}] = best_path;
    return best_path;
}

// This point_can_see_point checks against the base m_polygon_segments only.
bool ShortestPathCalculator::point_can_see_point(point_t p1, point_t p2) const {
    segment_t segment{p1, p2};
    for (const auto &border_segment: m_polygon_segments) {
        if (segments_intersect(segment, border_segment)) {
            return false;
        }
    }
    return true;
}

// This point_can_see_point checks against base m_polygon_segments AND additional_obstacle_polygons.
bool ShortestPathCalculator::point_can_see_point(point_t p1, point_t p2, const std::vector<polygon_t>& additional_obstacle_polygons) const {
    segment_t segment{p1, p2};
    // Check against base polygon segments
    for (const auto &border_segment: m_polygon_segments) {
        if (segments_intersect(segment, border_segment)) {
            return false;
        }
    }
    // Check against additional obstacle polygons
    for (const auto& poly : additional_obstacle_polygons) {
        for (size_t i = 0; i + 1 < poly.size(); ++i) {
            segment_t poly_segment = {poly[i], poly[i+1]};
            if (segments_intersect(segment, poly_segment)) {
                return false;
            }
        }
    }
    return true;
}
