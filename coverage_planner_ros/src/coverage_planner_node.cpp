// ROS 2 service front end of coverage_planner_core.
//
//   srv ~/compute_coverage_path   coverage_planner_ros/srv/ComputeCoveragePath
//
// Parameters:
//   config_file        YAML with the cost model and solver tuning (default: coverage_planner_core's default_planner.yaml)
//   latitude_origin    fallback origin of the internal metric frame when the request leaves it at 0,0
//   longitude_origin

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <coverage_planner_core/planner.hpp>
#include <coverage_planner_ros/srv/compute_coverage_path.hpp>

namespace coverage_planner_ros
{

using ComputeCoveragePath = coverage_planner_ros::srv::ComputeCoveragePath;

// Forwards the planner's logging into the ROS logger
class RosLogger : public loggers::SimpleLogger
{
public:
  explicit RosLogger(rclcpp::Logger logger) : logger_(std::move(logger)) { m_log_level = loggers::LOG_INFO; }
  void log_debug(const std::string& s) override { RCLCPP_DEBUG(logger_, "%s", s.c_str()); }
  void log_info(const std::string& s) override
  {
    if (m_log_level <= loggers::LOG_INFO) RCLCPP_INFO(logger_, "%s", s.c_str());
  }
  void log_warn(const std::string& s) override { RCLCPP_WARN(logger_, "%s", s.c_str()); }
  void log_err(const std::string& s) override { RCLCPP_ERROR(logger_, "%s", s.c_str()); }

private:
  rclcpp::Logger logger_;
};

class CoveragePlannerNode : public rclcpp::Node
{
public:
  explicit CoveragePlannerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("coverage_planner", options)
  {
    const std::string default_config = ament_index_cpp::get_package_share_directory("coverage_planner_core") + "/config/default_planner.yaml";
    config_file_ = declare_parameter<std::string>("config_file", default_config);
    origin_lat_ = declare_parameter<double>("latitude_origin", 0.0);
    origin_lon_ = declare_parameter<double>("longitude_origin", 0.0);

    coverage_planner_core::load_from_file(config_file_, model_, params_);
    RCLCPP_INFO(get_logger(), "loaded '%s': %s model, %d rotations, %d no-improvement cycles, time budget %.1f s", config_file_.c_str(),
                model_.optimization_type.c_str(), params_.number_of_rotations, params_.no_improvement_cycles_before_stop, params_.time_budget_s);

    logger_ = std::make_shared<RosLogger>(get_logger());
    srv_ = create_service<ComputeCoveragePath>("~/compute_coverage_path",
                                               [this](const std::shared_ptr<ComputeCoveragePath::Request> req,
                                                      std::shared_ptr<ComputeCoveragePath::Response> res) { handle(*req, *res); });
  }

private:
  static std::vector<coverage_planner_core::GeoPoint> to_geo(const coverage_planner_ros::msg::GeoPolygon& poly)
  {
    std::vector<coverage_planner_core::GeoPoint> out;
    for (const auto& p : poly.points) {
      out.push_back({p.x, p.y});
    }
    return out;
  }

  static double padded(const std::vector<double>& v, size_t i, double fallback)
  {
    if (v.empty()) {
      return fallback;
    }
    return i < v.size() ? v[i] : v.back();
  }

  void handle(const ComputeCoveragePath::Request& req, ComputeCoveragePath::Response& res)
  {
    coverage_planner_core::PlanRequest r;
    r.model = model_;
    r.params = params_;

    const bool req_origin = req.latitude_origin != 0.0 || req.longitude_origin != 0.0;
    r.origin = req_origin ? coverage_planner_core::GeoPoint{req.latitude_origin, req.longitude_origin}
                          : coverage_planner_core::GeoPoint{origin_lat_, origin_lon_};
    if (r.origin.lat == 0.0 && r.origin.lon == 0.0 && !req.initial_drone_positions.empty()) {
      r.origin = {req.initial_drone_positions[0].x, req.initial_drone_positions[0].y};  // any nearby point works as origin
    }

    for (const auto& z : req.fly_zones) {
      r.search_areas.push_back({to_geo(z), 0.0});
    }
    for (const auto& z : req.no_fly_zones) {
      r.no_fly_zones.push_back({to_geo(z), 0.0});
    }
    for (size_t i = 0; i < req.hr_no_fly_zones.size(); ++i) {
      if (i >= req.hr_no_fly_depths.size()) {
        res.success = false;
        res.message = "hr_no_fly_depths must have one ceiling per height-restricted zone";
        return;
      }
      r.no_fly_zones.push_back({to_geo(req.hr_no_fly_zones[i]), req.hr_no_fly_depths[i]});
    }

    for (size_t i = 0; i < req.initial_drone_positions.size(); ++i) {
      coverage_planner_core::DroneInput d;
      d.name = i < req.drone_names.size() ? req.drone_names[i] : "uav" + std::to_string(i + 1);
      d.start = {req.initial_drone_positions[i].x, req.initial_drone_positions[i].y};
      d.min_horizontal_distance = padded(req.min_horizontal_distances, i, 5.0);
      d.min_vertical_distance = padded(req.min_vertical_distances, i, 2.5);
      r.drones.push_back(d);
    }
    for (const auto& s : req.end_slots) {
      r.end_slots.push_back({s.x, s.y});
    }
    r.sweep_height = req.sweeping_height > 0.0 ? req.sweeping_height : 5.0;
    r.sweeping_step = req.sweeping_step;
    r.max_altitude = req.max_altitude;
    if (req.seed >= 0) {
      r.seed = static_cast<uint64_t>(req.seed);
    }

    RCLCPP_INFO(get_logger(), "planning for %zu drones over %zu areas (%zu no-fly zones), sweep %.1f m", r.drones.size(), r.search_areas.size(),
                r.no_fly_zones.size(), r.sweep_height);
    const auto t0 = now();
    const auto result = coverage_planner_core::plan(r, logger_);
    res.success = result.success;
    res.message = result.message;
    res.seed = result.seed;
    RCLCPP_INFO(get_logger(), "planning %s in %.2f s: %s", result.success ? "succeeded" : "FAILED", (now() - t0).seconds(), result.message.c_str());
    if (!result.success) {
      return;
    }

    for (const auto& d : result.drones) {
      mrs_msgs::msg::TrajectoryReference traj;
      traj.header.stamp = now();
      traj.header.frame_id = "latlon_origin";
      traj.use_heading = true;
      for (const auto& w : d.waypoints) {
        mrs_msgs::msg::Reference ref;
        ref.position.x = w.lat;
        ref.position.y = w.lon;
        ref.position.z = w.z;
        ref.heading = w.heading;
        traj.points.push_back(ref);
      }
      res.drone_names.push_back(d.name);
      res.drone_paths.push_back(traj);
      res.sweep_begin_indices.push_back(static_cast<uint32_t>(d.sweep_begin));
      res.sweep_end_indices.push_back(static_cast<uint32_t>(d.sweep_end));
      res.has_sweep.push_back(d.has_sweep);
      res.costs.push_back(d.cost);
    }
  }

  std::string config_file_;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  coverage_planner_core::DroneCostModel model_;
  coverage_planner_core::AlgorithmParams params_;
  std::shared_ptr<RosLogger> logger_;
  rclcpp::Service<ComputeCoveragePath>::SharedPtr srv_;
};

}  // namespace coverage_planner_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coverage_planner_ros::CoveragePlannerNode)
