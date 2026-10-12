# coverage_mission_planner_ros

Multi-UAV coverage path planning for ROS 2 (Jazzy): split one or more polygon areas among N drones so that the
slowest drone finishes as early as possible (time model) or uses the least energy (energy model), then add the
transits from each drone's start and to its end slot at deconflicted altitudes.

| Package | What it is |
| --- | --- |
| `coverage_planner_core` | ROS-free C++17 library (yaml-cpp only): the EnergyAwareMCPP solver in `lib/EnergyAwareMCPP` plus the planning pipeline in `src/`. Also `coverage_planner_cli` and gtests. |
| `coverage_planner_ros` | ROS 2 component/executable `coverage_planner` serving `~/compute_coverage_path` (`coverage_planner_ros/srv/ComputeCoveragePath`). |

## Origin of the code

- `lib/EnergyAwareMCPP` is the library from `iroc_fleet_manager` branch `feat/coverage-planner-upgrade` (time
  optimisation, `PathCostCalculator`, per-drone cost models), plus:
  - the bounded top-500 candidate heap in `MstspSolver::greedy_random` (from this repo's earlier ROS 1 version);
  - a per-solver random generator seeded from the request (reproducible plans, solvers can run concurrently);
  - a wall-clock `time_budget_s` for the tabu search;
  - `main()` moved out of the library into `tools/cli_main.cpp`.
- `src/planner.cpp` and `src/transit_heights.cpp` are the pipeline that used to live in iroc_fleet_manager's
  `CoveragePlanner` plugin (zone validation, solve, Hungarian assignment of paths to drones, transit paths,
  transit-height deconfliction), made ROS-free and reentrant, with these changes:
  - height-restricted zones whose ceiling is below the sweep altitude are ignored (the sweep flies over them);
  - zone polygons (search area, no-fly and height-restricted) may be given open or closed (last point repeating the first); the
    first point is appended only when it isn't already repeated (the plugin always appended it, so a closed
    input got a duplicate vertex and a zero-length edge);
  - the solver's common start point is the centroid of the drones' starts (the plugin used the first drone's
    position);
  - one cost model for all drones (the plugin loaded one per drone);
  - per-drone minimum distances stay with their drone (the plugin reordered them by path index);
  - optional `end_slots`: drones end at assigned slots instead of returning to their start;
  - a drone the solver gives no area gets a plain transit (`has_sweep = false`) instead of failing the plan;
  - `max_altitude` fails the plan on the planner instead of later in trajectory validation.

## API

```cpp
#include <coverage_planner_core/planner.hpp>

coverage_planner_core::PlanRequest req;
req.origin = {49.2283, 15.2252};
req.search_areas = {{{{49.2284, 15.2253}, {49.2284, 15.2259}, {49.2288, 15.2259}, {49.2288, 15.2253}}}};
req.drones = {{"uav1", {49.2282, 15.2251}}, {"uav2", {49.2282, 15.2252}}};
req.sweep_height = 6.0;
coverage_planner_core::load_from_file(".../default_planner.yaml", req.model, req.params);
auto res = coverage_planner_core::plan(req);   // res.drones[i].name, .waypoints (lat, lon, z), .sweep_begin/.sweep_end
```

Results are matched to drones **by name**: the solver plans all paths from one start point and assigns them to
drones afterwards. All drones share one cost model.

## Build and test

```bash
colcon build --packages-up-to coverage_planner_ros
colcon test --packages-select coverage_planner_core && colcon test-result --verbose
```

## Run

```bash
ros2 launch coverage_planner_ros coverage_planner.launch.py            # config_file:=<yaml>
ros2 run coverage_planner_ros plan_client.py --drones 4 --end-slots --png /tmp/plan.png
ros2 run coverage_planner_core coverage_planner_cli <sample_config>.yaml # writes path_<i>.csv
```

Configuration: `coverage_planner_core/config/default_planner.yaml` (same layout as iroc_fleet_manager's
`coverage_planner_config.yaml`). Sample problems and a plot script are in `lib/EnergyAwareMCPP/sample_configs` and
`lib/EnergyAwareMCPP/scripts/visualize_paths.py`.
