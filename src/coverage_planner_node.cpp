#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <mrs_coverage_planner/ComputeCoveragePath.h>
#include <mrs_coverage_planner/coverage_planner_core.h>
#include <mrs_lib/param_loader.h>
#include <EnergyAwareMCPP/coverage_planner.hpp>

#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>

#include <vector>
#include <string>

class CoveragePlannerNode {
public:
    CoveragePlannerNode() : nh_("~") {
        mrs_lib::ParamLoader param_loader(nh_);
        
        // 1. Load Algorithm Config
        planner_config_.lat_lon_origin = {
            param_loader.loadParam2<double>("latitude_origin", 49.228330), 
            param_loader.loadParam2<double>("longitude_origin", 15.225238)
        };
        planner_config_.points_in_lat_lon = param_loader.loadParam2<bool>("points_in_lat_lon", true);
        planner_config_.sweeping_step = param_loader.loadParam2<double>("sweeping_step", 1.0);
        planner_config_.number_of_drones = param_loader.loadParam2<int>("number_of_drones", 1);
        planner_config_.number_of_rotations = param_loader.loadParam2<int>("number_of_rotations", 3);
        
        // 2. Load Energy Calculator Config
        auto& ec = planner_config_.energy_calculator_config;
        ec.drone_mass = param_loader.loadParam2<double>("drone_mass", 3.2);
        ec.propeller_radius = param_loader.loadParam2<double>("propeller_radius", 0.19);
        ec.number_of_propellers = param_loader.loadParam2<int>("number_of_propellers", 4);
        ec.average_acceleration = param_loader.loadParam2<double>("average_acceleration", 2.0);
        ec.drone_area = param_loader.loadParam2<double>("drone_area", 0.07);
        ec.allowed_path_deviation = param_loader.loadParam2<double>("allowed_path_deviation", 2.0);
        
        ec.battery_model.cell_capacity = param_loader.loadParam2<double>("cell_capacity", 5.0);
        ec.battery_model.number_of_cells = param_loader.loadParam2<int>("number_of_cells", 4);
        ec.battery_model.d0 = param_loader.loadParam2<double>("d0", 0.99876);
        ec.battery_model.d1 = param_loader.loadParam2<double>("d1", -0.0020);
        ec.battery_model.d2 = param_loader.loadParam2<double>("d2", -5.2484e-05);
        ec.battery_model.d3 = param_loader.loadParam2<double>("d3", 1.2230e-07);

        ec.best_speed_model.c0 = param_loader.loadParam2<double>("c0", 0.041546);
        ec.best_speed_model.c1 = param_loader.loadParam2<double>("c1", 0.041122);
        ec.best_speed_model.c2 = param_loader.loadParam2<double>("c2", 0.00053292);
        
        service_ = nh_.advertiseService("compute_coverage_path", &CoveragePlannerNode::handleComputePathRequest, this);
        ROS_INFO("[CoveragePlannerNode]: Service initialized.");
    }

    bool handleComputePathRequest(mrs_coverage_planner::ComputeCoveragePath::Request &req,
                                 mrs_coverage_planner::ComputeCoveragePath::Response &res) {
        ROS_INFO("[CoveragePlannerNode]: Received incoming planning request.");

        try {
            // 1. Convert Service Request to Core Types
            std::vector<mrs_coverage_planner::polygon_t> fly_zones;
            for (const auto& zone : req.fly_zones) {
                mrs_coverage_planner::polygon_t poly;
                for (const auto& p : zone.points) {
                    poly.push_back({p.x, p.y});
                }
                fly_zones.push_back(poly);
            }

            std::vector<mrs_coverage_planner::polygon_t> no_fly_zones;
            for (const auto& poly : req.no_fly_zones) {
                mrs_coverage_planner::polygon_t nfz;
                for (const auto& p : poly.points) {
                    nfz.push_back({p.x, p.y});
                }
                no_fly_zones.push_back(nfz);
            }

            // 2. Prepare Planner Configuration
            algorithm_config_t config = planner_config_;
            config.sweeping_alt = req.target_sweeping_height; 
            config.decomposition_type = BOUSTROPHEDON_DECOMPOSITION;
            config.min_sub_polygons_per_uav = 1;

            EnergyCalculator energy_calc(config.energy_calculator_config);
            config.max_single_path_energy = energy_calc.get_hover_power() * 3600.0; 

            config.number_of_drones = req.initial_drone_positions.size() > 0 ? 
                                     req.initial_drone_positions.size() : config.number_of_drones;

            std::vector<mrs_coverage_planner::point_t> initial_positions;
            for (const auto& p : req.initial_drone_positions) {
                initial_positions.push_back({p.x, p.y});
            }
            if (initial_positions.empty() && !fly_zones.empty()) {
                initial_positions.push_back(fly_zones[0][0]);
            } else if (initial_positions.empty()) {
                initial_positions.push_back({0.0, 0.0});
            }

            // Map altitude & horizontal deconfliction vectors from request
            std::vector<double> min_horiz = req.min_horizontal_distances;
            std::vector<double> min_vert = req.min_vertical_distances;

            if (min_horiz.size() < initial_positions.size()) {
                min_horiz.resize(initial_positions.size(), min_horiz.empty() ? 5.0 : min_horiz.back());
            }
            if (min_vert.size() < initial_positions.size()) {
                min_vert.resize(initial_positions.size(), min_vert.empty() ? 5.0 : min_vert.back());
            }

            // 3. Call the Core Planning Algorithm
            auto paths = mrs_coverage_planner::planStandaloneMission(
                initial_positions,
                fly_zones,
                no_fly_zones,
                {}, 
                min_horiz,
                min_vert,
                config,
                req.target_sweeping_height
            );

            if (paths.empty()) {
                res.success = false;
                res.message = "No paths were computed by the planner.";
                return true;
            }

            // 4. Convert Core Results back to GPS Trajectories (Standardized ROS Layout)
            for (const auto& path : paths) {
                mrs_msgs::TrajectoryReference trajectory;
                trajectory.header.stamp = ros::Time::now();
                trajectory.header.frame_id = "gps";
                trajectory.fly_now = true;
                trajectory.use_heading = true;

                for (const auto& wp : path) {
                    mrs_msgs::Reference reference;
                    // Core algorithm returns the final coordinates in Lat/Lon if config.points_in_lat_lon was true
                    // Note: The core algorithm's output is in the same coordinate system as the input, so if you provided GPS coordinates, it will return GPS coordinates.
                    reference.position.x = wp.position.y; // Latitude
                    reference.position.y = wp.position.x; // Longitude
                    reference.position.z = wp.position.z; // De-conflicted altitude layers from core
                    reference.heading = wp.heading;
                    trajectory.points.push_back(reference);
                }
                res.drone_paths.push_back(trajectory);
            }

            res.success = true;
            res.message = "Path successfully computed.";
        } catch (const std::exception& e) {
            ROS_ERROR("[CoveragePlannerNode]: Planning failed: %s", e.what());
            res.success = false;
            res.message = std::string("Planning failed: ") + e.what();
        }

        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    algorithm_config_t planner_config_{};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mrs_coverage_planner_node");
    CoveragePlannerNode node;
    ros::spin();
    return 0;
}