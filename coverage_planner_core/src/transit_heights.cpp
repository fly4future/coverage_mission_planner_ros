#include "transit_heights.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace coverage_planner_core::detail
{

namespace
{

struct Pt
{
  double x, y;
};

double dist_sq(Pt a, Pt b)
{
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

double point_to_segment_distance(Pt p, Pt s1, Pt s2)
{
  const double l2 = dist_sq(s1, s2);
  if (l2 == 0.0) {
    return std::sqrt(dist_sq(p, s1));
  }
  double t = ((p.x - s1.x) * (s2.x - s1.x) + (p.y - s1.y) * (s2.y - s1.y)) / l2;
  t = std::clamp(t, 0.0, 1.0);
  return std::sqrt(dist_sq(p, {s1.x + t * (s2.x - s1.x), s1.y + t * (s2.y - s1.y)}));
}

// true when the segments cross or come closer than min_distance
bool check_overlap(const TransitPath& a, const TransitPath& b, double min_distance)
{
  if (segments_intersect(a, b)) {
    return true;
  }
  const Pt a1{a.x1, a.y1}, a2{a.x2, a.y2}, b1{b.x1, b.y1}, b2{b.x2, b.y2};
  const double d = std::min({point_to_segment_distance(a1, b1, b2), point_to_segment_distance(a2, b1, b2), point_to_segment_distance(b1, a1, a2),
                             point_to_segment_distance(b2, a1, a2)});
  return d < min_distance;
}

bool check_overlap(const TransitPathGroup& g1, const TransitPathGroup& g2, double min_distance)
{
  for (const auto& a : g1.get()) {
    for (const auto& b : g2.get()) {
      if (check_overlap(a, b, min_distance)) {
        return true;
      }
    }
  }
  return false;
}

bool check_overlap(const TransitPathGroup& g, const TransitPath& tp, double min_distance)
{
  for (const auto& a : g.get()) {
    if (check_overlap(a, tp, min_distance)) {
      return true;
    }
  }
  return false;
}

// -1 => g1 should be above g2, 0 => no constraint, 1 => g2 should be above g1.
// A group must pass above another one whose start or end (where it climbs or descends) is close to it.
int vertical_order(const TransitPathGroup& g1, const TransitPathGroup& g2, double drone_distance)
{
  const auto endpoint_close = [drone_distance](const TransitPathGroup& over, const TransitPathGroup& under) {
    const Pt start{under.get().front().x1, under.get().front().y1};
    const Pt end{under.get().back().x2, under.get().back().y2};
    for (const auto& tp : over.get()) {
      const Pt s{tp.x1, tp.y1}, e{tp.x2, tp.y2};
      if (point_to_segment_distance(start, s, e) < drone_distance || point_to_segment_distance(end, s, e) < drone_distance) {
        return true;
      }
    }
    return false;
  };
  const bool g1_above_g2 = endpoint_close(g1, g2);
  const bool g2_above_g1 = endpoint_close(g2, g1);
  if (g1_above_g2 == g2_above_g1) {
    return 0;
  }
  return g1_above_g2 ? -1 : 1;
}

bool creates_cycle(const std::vector<std::vector<int>>& under, int start, int search)
{
  for (int idx : under.at(start)) {
    if (idx == search || creates_cycle(under, idx, search)) {
      return true;
    }
  }
  return false;
}

struct NodePriority
{
  int id;
  int degree;
  double best_available_height;

  // more overlaps first; on a tie, the one that can stay lower
  bool operator>(const NodePriority& o) const
  {
    if (degree != o.degree) {
      return degree > o.degree;
    }
    return best_available_height < o.best_available_height;
  }
};

}  // namespace

bool segments_intersect(const TransitPath& a, const TransitPath& b)
{
  const auto ccw = [](double ax, double ay, double bx, double by, double cx, double cy) { return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax); };
  return (ccw(a.x1, a.y1, b.x1, b.y1, b.x2, b.y2) != ccw(a.x2, a.y2, b.x1, b.y1, b.x2, b.y2)) &&
         (ccw(a.x1, a.y1, a.x2, a.y2, b.x1, b.y1) != ccw(a.x1, a.y1, a.x2, a.y2, b.x2, b.y2));
}

void TransitPathGroup::addTransitPath(const TransitPath& tp, double* z1, double* z2)
{
  paths_.push_back(tp);
  for (double* z : {z1, z2}) {
    if (std::find(z_ptrs_.begin(), z_ptrs_.end(), z) == z_ptrs_.end()) {
      z_ptrs_.push_back(z);
    }
  }
}

void resolve_transit_heights(std::vector<path_t>& paths, double sweeping_height, double transit_height,
                             const std::vector<double>& min_horizontal_distances, const std::vector<double>& min_vertical_distances)
{
  // 1. Collect the transit path groups: maximal runs of segments with both ends above the sweep altitude
  std::vector<std::unique_ptr<TransitPathGroup>> groups;
  for (size_t i = 0; i < paths.size(); ++i) {
    auto& path = paths[i];
    for (size_t j = 1; j < path.size(); ++j) {
      auto& cur = path[j];
      auto& prev = path[j - 1];
      if (!(cur.z > sweeping_height && prev.z > sweeping_height) || (cur.x == prev.x && cur.y == prev.y)) {
        continue;
      }
      const TransitPath tp{prev.x, prev.y, cur.x, cur.y};
      const bool extends_last = !groups.empty() && groups.back()->drone_idx == static_cast<int>(i) && groups.back()->get().back().x2 == prev.x &&
                                groups.back()->get().back().y2 == prev.y;
      if (extends_last) {
        groups.back()->addTransitPath(tp, &prev.z, &cur.z);
      } else {
        auto g = std::make_unique<TransitPathGroup>(static_cast<int>(i), min_horizontal_distances.at(i), min_vertical_distances.at(i));
        g->addTransitPath(tp, &prev.z, &cur.z);
        g->setHeight(cur.z, sweeping_height, transit_height);
        groups.push_back(std::move(g));
      }
    }
  }
  const int n = static_cast<int>(groups.size());
  if (n == 0) {
    return;
  }

  // 2. Which group must be above which (where one climbs/descends right next to another), without cycles
  std::vector<std::vector<int>> under(n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (groups[i]->drone_idx == groups[j]->drone_idx) {
        continue;
      }
      const double d = std::max(min_horizontal_distances.at(groups[i]->drone_idx), min_horizontal_distances.at(groups[j]->drone_idx));
      const int r = vertical_order(*groups[i], *groups[j], d);
      if (r == -1 && !creates_cycle(under, j, i)) {
        under[i].push_back(j);
      } else if (r == 1 && !creates_cycle(under, i, j)) {
        under[j].push_back(i);
      }
    }
  }

  // 3. Overlap graph between groups of different drones
  std::vector<std::vector<int>> adj(n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (groups[i]->drone_idx == groups[j]->drone_idx) {
        continue;
      }
      const double d = std::max(min_horizontal_distances.at(groups[i]->drone_idx), min_horizontal_distances.at(groups[j]->drone_idx));
      if (check_overlap(*groups[i], *groups[j], d)) {
        adj[i].push_back(j);
        adj[j].push_back(i);
      }
    }
  }

  // 4. Greedy colouring: repeatedly pick the highest-priority unprocessed group and give it the lowest safe altitude
  std::vector<double> assigned(n, -1.0);
  std::vector<bool> processed(n, false);
  for (int step = 0; step < n; ++step) {
    int best_node = -1;
    NodePriority best{-1, -1, std::numeric_limits<double>::max()};

    for (int v = 0; v < n; ++v) {
      if (processed[v]) {
        continue;
      }
      // a group that must be above something not yet placed has to wait
      if (std::any_of(under[v].begin(), under[v].end(), [&](int u) { return !processed[u]; })) {
        continue;
      }
      const auto& gv = *groups[v];
      double h = std::max(sweeping_height, gv.drone_height);
      for (int u : under[v]) {
        h = std::max(h, assigned[u] + std::max(gv.min_vertical_distance, groups[u]->min_vertical_distance));
      }
      // a group at sweep level crossing another drone's sweep lanes goes above them
      if (h == sweeping_height) {
        for (size_t k = 0; k < paths.size(); ++k) {
          if (static_cast<int>(k) == gv.drone_idx) {
            continue;
          }
          for (size_t j = 1; j < paths[k].size(); ++j) {
            const auto& c = paths[k][j];
            const auto& p = paths[k][j - 1];
            if (c.z == sweeping_height && p.z == sweeping_height &&
                check_overlap(gv, TransitPath{c.x, c.y, p.x, p.y}, std::max(gv.min_horizontal_distance, min_horizontal_distances.at(k)))) {
              h = std::max(h, sweeping_height + std::max(gv.min_vertical_distance, min_vertical_distances.at(k)));
            }
          }
        }
      }
      // keep the vertical separation to already placed overlapping groups
      for (int nb : adj[v]) {
        const double sep = std::max(gv.min_vertical_distance, min_vertical_distances.at(groups[nb]->drone_idx));
        if (assigned[nb] != -1.0 && std::abs(h - assigned[nb]) < sep) {
          h = assigned[nb] + sep;
        }
      }
      const NodePriority cur{v, static_cast<int>(adj[v].size()), h};
      if (best_node == -1 || cur > best) {
        best = cur;
        best_node = v;
      }
    }
    if (best_node == -1) {
      break;
    }
    assigned[best_node] = best.best_available_height;
    processed[best_node] = true;
    groups[best_node]->drone_height = assigned[best_node];
    groups[best_node]->writeHeights(assigned[best_node]);
  }
}

std::vector<int> hungarian(const std::vector<std::vector<double>>& matrix)
{
  if (matrix.empty()) {
    return {};
  }
  const double inf = std::numeric_limits<double>::max();
  const int n = static_cast<int>(matrix.size());
  const int m = static_cast<int>(matrix[0].size());
  std::vector<double> u(n + 1, 0), v(m + 1, 0), minv(m + 1, 0);
  std::vector<int> p(m + 1, 0), way(m + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::fill(minv.begin(), minv.end(), inf);
    std::vector<bool> used(m + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      int j1 = 0;
      double delta = inf;
      for (int j = 1; j <= m; ++j) {
        if (!used[j]) {
          const double cur = matrix[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= m; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> result(n, -1);
  for (int j = 1; j <= m; ++j) {
    if (p[j] != 0) {
      result[p[j] - 1] = j - 1;
    }
  }
  return result;
}

}  // namespace coverage_planner_core::detail
