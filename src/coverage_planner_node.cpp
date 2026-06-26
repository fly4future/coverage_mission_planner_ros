#include <ros/ros.h>
#include <ros/package.h>
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
        param_loader.addYamlFile(ros::package::getPath("mrs_coverage_planner") + "/config/coverage_planner_config.yaml");
        planner_config_ = parse_algorithm_config(param_loader);
        if (!param_loader.loadedSuccessfully()) {
            ROS_ERROR("[CoveragePlannerNode]: Could not load all parameters!");
            exit(1);
        }
        
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

    // Maybe move to lib?:
    algorithm_config_t parse_algorithm_config(mrs_lib::ParamLoader &param_loader) const {
    const std::string yaml_prefix = "fleet_manager/planners/coverage_planner/";
    algorithm_config_t algorithm_config;

    // Load basic drone parameters
    param_loader.loadParam(yaml_prefix + "drone_mass", algorithm_config.energy_calculator_config.drone_mass);
    param_loader.loadParam(yaml_prefix + "drone_area", algorithm_config.energy_calculator_config.drone_area);
    param_loader.loadParam(yaml_prefix + "average_acceleration", algorithm_config.energy_calculator_config.average_acceleration);
    param_loader.loadParam(yaml_prefix + "propeller_radius", algorithm_config.energy_calculator_config.propeller_radius);
    param_loader.loadParam(yaml_prefix + "number_of_propellers", algorithm_config.energy_calculator_config.number_of_propellers);
    param_loader.loadParam(yaml_prefix + "allowed_path_deviation", algorithm_config.energy_calculator_config.allowed_path_deviation);
    param_loader.loadParam(yaml_prefix + "number_of_rotations", algorithm_config.number_of_rotations);

    // Load battery model parameters
    const std::string battery_prefix = yaml_prefix + "battery_model/";
    param_loader.loadParam(battery_prefix + "cell_capacity", algorithm_config.energy_calculator_config.battery_model.cell_capacity);
    param_loader.loadParam(battery_prefix + "number_of_cells", algorithm_config.energy_calculator_config.battery_model.number_of_cells);
    param_loader.loadParam(battery_prefix + "d0", algorithm_config.energy_calculator_config.battery_model.d0);
    param_loader.loadParam(battery_prefix + "d1", algorithm_config.energy_calculator_config.battery_model.d1);
    param_loader.loadParam(battery_prefix + "d2", algorithm_config.energy_calculator_config.battery_model.d2);
    param_loader.loadParam(battery_prefix + "d3", algorithm_config.energy_calculator_config.battery_model.d3);

    // Load speed model parameters
    const std::string speed_prefix = yaml_prefix + "best_speed_model/";
    param_loader.loadParam(speed_prefix + "c0", algorithm_config.energy_calculator_config.best_speed_model.c0);
    param_loader.loadParam(speed_prefix + "c1", algorithm_config.energy_calculator_config.best_speed_model.c1);
    param_loader.loadParam(speed_prefix + "c2", algorithm_config.energy_calculator_config.best_speed_model.c2);

    // Load coordinate system parameters
    param_loader.loadParam(yaml_prefix + "points_in_lat_lon", algorithm_config.points_in_lat_lon);
    if (algorithm_config.points_in_lat_lon) {
        param_loader.loadParam(yaml_prefix + "latitude_origin", algorithm_config.lat_lon_origin.first);
        param_loader.loadParam(yaml_prefix + "longitude_origin", algorithm_config.lat_lon_origin.second);
    }

    param_loader.loadParam(yaml_prefix + "sweeping_step", algorithm_config.sweeping_step);

    int decomposition_method;
    param_loader.loadParam(yaml_prefix + "decomposition_method", decomposition_method);
    algorithm_config.decomposition_type = static_cast<decomposition_type_t>(decomposition_method);

    param_loader.loadParam(yaml_prefix + "min_sub_polygons_per_uav", algorithm_config.min_sub_polygons_per_uav);

    // Load optimization parameters
    param_loader.loadParam(yaml_prefix + "rotations_per_cell", algorithm_config.rotations_per_cell);
    param_loader.loadParam(yaml_prefix + "no_improvement_cycles_before_stop", algorithm_config.no_improvement_cycles_before_stop);
    param_loader.loadParam(yaml_prefix + "max_single_path_energy", algorithm_config.max_single_path_energy);

    return algorithm_config;
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


