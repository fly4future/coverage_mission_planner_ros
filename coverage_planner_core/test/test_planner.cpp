#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <set>

#include <EnergyAwareMCPP/utils.hpp>
#include <coverage_planner_core/planner.hpp>

using namespace coverage_planner_core;

namespace
{

const point_t kOrigin{49.228330, 15.225238};

GeoPoint geo(double x, double y)
{
  const point_t ll = meters_to_gps_coordinates({x, y}, kOrigin);
  return {ll.first, ll.second};
}

point_t local(double lat, double lon)
{
  return gps_coordinates_to_meters({lat, lon}, kOrigin);
}

Zone rect(double x0, double y0, double x1, double y1)
{
  return Zone{{geo(x0, y0), geo(x1, y0), geo(x1, y1), geo(x0, y1)}, 0.0};
}

bool inside(const point_t& p, const std::vector<point_t>& poly)
{
  bool in = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].second > p.second) != (poly[j].second > p.second)) &&
        (p.first < (poly[j].first - poly[i].first) * (p.second - poly[i].second) / (poly[j].second - poly[i].second) + poly[i].first)) {
      in = !in;
    }
  }
  return in;
}

std::vector<point_t> local_poly(const Zone& z)
{
  std::vector<point_t> out;
  for (const auto& p : z.polygon) {
    out.push_back(local(p.lat, p.lon));
  }
  return out;
}

PlanRequest base_request(size_t n_drones)
{
  PlanRequest r;
  r.origin = {kOrigin.first, kOrigin.second};
  r.search_areas = {rect(0, 0, 120, 80)};
  r.sweep_height = 6.0;
  r.sweeping_step = 8.0;
  r.model.optimization_type = "time";
  r.model.time = {4.0, 2.0, 2.0};
  r.params.number_of_rotations = 2;
  r.params.no_improvement_cycles_before_stop = 40;
  r.seed = 42;
  for (size_t i = 0; i < n_drones; ++i) {
    DroneInput d;
    d.name = "uav" + std::to_string(i + 1);
    d.start = geo(-20.0, 4.0 * static_cast<double>(i));
    d.min_vertical_distance = 2.5;
    r.drones.push_back(d);
  }
  return r;
}

const DronePlan* find(const PlanResult& res, const std::string& name)
{
  for (const auto& d : res.drones) {
    if (d.name == name) {
      return &d;
    }
  }
  return nullptr;
}

}  // namespace

TEST(Planner, EveryDroneGetsANamedPlanWithSweep)
{
  const auto req = base_request(4);
  const auto res = plan(req);
  ASSERT_TRUE(res.success) << res.message;
  ASSERT_EQ(res.drones.size(), 4u);
  for (const auto& d : req.drones) {
    const auto* p = find(res, d.name);
    ASSERT_NE(p, nullptr) << d.name;
    EXPECT_TRUE(p->has_sweep);
    EXPECT_LT(p->sweep_begin, p->sweep_end);
    EXPECT_LE(p->sweep_end, p->waypoints.size());
    EXPECT_GT(p->cost, 0.0);
  }
}

TEST(Planner, SameSeedGivesSamePlan)
{
  const auto req = base_request(3);
  const auto a = plan(req);
  const auto b = plan(req);
  ASSERT_TRUE(a.success && b.success);
  EXPECT_EQ(a.seed, b.seed);
  for (const auto& da : a.drones) {
    const auto* db = find(b, da.name);
    ASSERT_NE(db, nullptr);
    ASSERT_EQ(da.waypoints.size(), db->waypoints.size());
    for (size_t k = 0; k < da.waypoints.size(); ++k) {
      EXPECT_DOUBLE_EQ(da.waypoints[k].lat, db->waypoints[k].lat);
      EXPECT_DOUBLE_EQ(da.waypoints[k].lon, db->waypoints[k].lon);
      EXPECT_DOUBLE_EQ(da.waypoints[k].z, db->waypoints[k].z);
    }
  }
}

TEST(Planner, SweepStaysInsideAreaAndOutOfNoFlyZone)
{
  auto req = base_request(3);
  req.no_fly_zones = {rect(50, 30, 70, 50)};
  const auto res = plan(req);
  ASSERT_TRUE(res.success) << res.message;
  const auto area = local_poly(req.search_areas[0]);
  const auto nfz = local_poly(req.no_fly_zones[0]);
  for (const auto& d : res.drones) {
    for (size_t k = d.sweep_begin; k < d.sweep_end; ++k) {
      const auto p = local(d.waypoints[k].lat, d.waypoints[k].lon);
      if (d.waypoints[k].z != req.sweep_height) {
        continue;  // transit hops between two cells of the same drone (lifted above the sweep) may leave the area
      }
      EXPECT_TRUE(inside(p, area)) << d.name << " sweep point outside the area (" << p.first << ", " << p.second << ")";
      EXPECT_FALSE(inside(p, nfz)) << d.name << " sweep point inside the no-fly zone (" << p.first << ", " << p.second << ")";
    }
  }
}

TEST(Planner, PathsEndAtTheirAssignedSlots)
{
  auto req = base_request(4);
  for (size_t i = 0; i < 4; ++i) {
    req.end_slots.push_back(geo(140.0, 10.0 * static_cast<double>(i)));
  }
  const auto res = plan(req);
  ASSERT_TRUE(res.success) << res.message;
  std::set<size_t> used;
  for (const auto& d : res.drones) {
    ASSERT_TRUE(d.end_slot.has_value());
    used.insert(*d.end_slot);
    const auto end = local(d.waypoints.back().lat, d.waypoints.back().lon);
    const auto slot = local(req.end_slots[*d.end_slot].lat, req.end_slots[*d.end_slot].lon);
    EXPECT_NEAR(end.first, slot.first, 0.05);
    EXPECT_NEAR(end.second, slot.second, 0.05);
    // the first waypoint is the drone's own start
    const auto* in = &req.drones[0];
    for (const auto& di : req.drones) {
      if (di.name == d.name) in = &di;
    }
    const auto first = local(d.waypoints.front().lat, d.waypoints.front().lon);
    const auto start = local(in->start.lat, in->start.lon);
    EXPECT_NEAR(first.first, start.first, 0.05);
    EXPECT_NEAR(first.second, start.second, 0.05);
  }
  EXPECT_EQ(used.size(), 4u);
}

TEST(Planner, WithoutReturnPathsEndWithTheSweep)
{
  auto req = base_request(3);
  req.return_to_start = false;
  const auto res = plan(req);
  ASSERT_TRUE(res.success) << res.message;
  for (const auto& d : res.drones) {
    ASSERT_TRUE(d.has_sweep);
    EXPECT_FALSE(d.end_slot.has_value());
    // nothing after the sweep but its own ascent point
    EXPECT_LE(d.waypoints.size(), d.sweep_end + 1) << d.name;
  }
}

TEST(Planner, TransitsNeverDipBelowSweepAndAltitudeLimitIsEnforced)
{
  auto req = base_request(4);
  const auto ok = plan(req);
  ASSERT_TRUE(ok.success) << ok.message;
  double max_z = 0.0;
  for (const auto& d : ok.drones) {
    for (size_t k = 0; k < d.waypoints.size(); ++k) {
      max_z = std::max(max_z, d.waypoints[k].z);
      // conflict-free transits may fly at sweep altitude, conflicting ones are lifted above it
      EXPECT_GE(d.waypoints[k].z, req.sweep_height) << d.name;
      if (k >= d.sweep_begin && k < d.sweep_end && k > d.sweep_begin && k + 1 < d.sweep_end) {
        continue;
      }
    }
  }
  ASSERT_GT(max_z, req.sweep_height) << "expected at least one lifted transit with 4 drones";
  req.max_altitude = req.sweep_height + 0.5;  // below the lifted transits -> must fail on the planner, not later
  const auto limited = plan(req);
  EXPECT_FALSE(limited.success);
  EXPECT_NE(limited.message.find("max_altitude"), std::string::npos) << limited.message;
}

TEST(Planner, RejectsBadRequests)
{
  auto req = base_request(2);
  req.end_slots = {geo(0, 0)};
  EXPECT_FALSE(plan(req).success);

  req = base_request(2);
  req.drones[1].name = req.drones[0].name;
  EXPECT_FALSE(plan(req).success);

  req = base_request(2);
  req.no_fly_zones = {rect(-10, -10, 10, 10)};  // straddles the search-area corner
  EXPECT_FALSE(plan(req).success);
}

TEST(Planner, LoadsFleetManagerStyleConfig)
{
  DroneCostModel model;
  AlgorithmParams params;
  ASSERT_NO_THROW(load_from_file(std::string(SAMPLE_CONFIGS_DIR) + "/multiple-fly-zones/config.yaml", model, params));
  EXPECT_EQ(model.optimization_type, "time");
  EXPECT_DOUBLE_EQ(model.time.max_speed, 10.0);
  EXPECT_EQ(params.number_of_rotations, 3);
  EXPECT_EQ(params.no_improvement_cycles_before_stop, 400);
}

// Session-sized task (40 x 40 m, 4 drones, default tuning): must stay fast enough to plan on board.
TEST(Planner, SessionSizedPlanIsFast)
{
  auto req = base_request(4);
  req.search_areas = {rect(-20, -20, 20, 20)};
  req.sweeping_step = 6.0;
  req.params = AlgorithmParams{};
  for (size_t i = 0; i < req.drones.size(); ++i) {
    req.drones[i].start = geo(-30.0, -6.0 + 4.0 * static_cast<double>(i));
  }
  const auto t0 = std::chrono::steady_clock::now();
  const auto res = plan(req);
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_TRUE(res.success) << res.message;
  RecordProperty("plan_time_s", std::to_string(dt));
  std::cout << "[ SESSION  ] 4 drones, 40x40 m, default params: " << dt << " s" << std::endl;
  EXPECT_LT(dt, 60.0);
}

TEST(Planner, TimeBudgetBoundsTheSolve)
{
  auto req = base_request(4);
  req.params.no_improvement_cycles_before_stop = 100000;  // would run for a very long time
  req.params.time_budget_s = 2.0;
  const auto t0 = std::chrono::steady_clock::now();
  const auto res = plan(req);
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_TRUE(res.success) << res.message;
  EXPECT_LT(dt, 2.0 + 10.0);  // budget + greedy init + post-processing
}
