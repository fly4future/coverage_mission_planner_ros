#ifndef THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H
#define THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H

#include "SolverConfig.h"
#include "TargetSet.h"
#include <vector>
#include "MapPolygon.hpp"
#include "Target.h"
#include "ShortestPathCalculator.hpp"
#include "custom_types.hpp"
#include "PathCostCalculator.hpp"
#include <SimpleLogger.h>

struct metaheuristic_application_error : public std::runtime_error {
    using runtime_error::runtime_error;
};


struct solution_cost_t {
    /*!
     * Struct for representation the cost of one solution
     */
    double max_path_cost;
    double path_cost_sum;

   // Add explicit constructor
    solution_cost_t(double max_cost, double sum_cost) 
        : max_path_cost(max_cost), path_cost_sum(sum_cost) {}

    static solution_cost_t max() {
        return {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    }

    static solution_cost_t min() {
        return {std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
    }

    bool operator<(const solution_cost_t &rhs) const {
        return (max_path_cost < rhs.max_path_cost ||
                (max_path_cost == rhs.max_path_cost && path_cost_sum < rhs.path_cost_sum));
    }
};

/*!
 * TODO: move this function from here
 * Remove the heading from path and convert it into coordinates on one 2d plane
 * @param init Initial path
 * @return path in a plane without heading
 */
std::vector<std::pair<double, double>> remove_path_heading(const std::vector<point_heading_t<double>> &init);

namespace mstsp_solver {

    using _instance_solution_t = std::vector<std::vector<Target>>;

    /*!
     * Struct representing the final result of the solver
     */
    struct final_solution_t {
        double max_path_energy;
        double path_energies_sum;
        std::vector<std::vector<point_heading_t < double>>> paths;
    };



    class MstspSolver {

    public:
        MstspSolver(SolverConfig config,
                    const std::vector<MapPolygon> &decomposed_polygons,
                    std::vector<std::shared_ptr<PathCostCalculator>> cost_calculators,
                    ShortestPathCalculator shortest_path_calculator);

        /*!
         * produce the solution of entire problem
         * @return pair of solution cost and solution itself in a form of vector of paths where each path
         * is a sequence of waypoints with heading
         */
        final_solution_t solve() const;

        void set_logger(std::shared_ptr<loggers::SimpleLogger> new_logger) {
            m_logger = std::move(new_logger);
        }

    private:
        std::shared_ptr<loggers::SimpleLogger> m_logger;
        std::vector<TargetSet> m_target_sets;
        const SolverConfig m_config;
        std::vector<std::shared_ptr<PathCostCalculator>> m_cost_calculators;
        const ShortestPathCalculator m_shortest_path_calculator;
        double m_cost_constant = 0.0001;

        /*!
         * Generate a solution using a greedy random method
         * @return greedy solution
         */
        _instance_solution_t greedy_random() const;

        /*!
         * Get the estimated cost of the path
         * @param path Sequence of targets, cost for which should be calculated
         * @param uav_idx Index of the UAV for which the cost is calculated
         * @return Path cost
         */
        double get_path_cost(const std::vector<Target> &path, size_t uav_idx) const;

        /*!
         * @param solution Problem solution
         * @return Cost of the solution
         */
        solution_cost_t get_solution_cost(const _instance_solution_t &solution) const;

        /*!
         * Gwt paths for all the drones from the specified problem solution
         * @param solution  Problem solution as path consisting of Targets that need to be visited by each drone
         * @return Vector of paths for each drone (size if config.n_uavs)
         */
        std::vector<std::vector<point_heading_t < double>>>

        get_drones_paths(const _instance_solution_t &solution) const;

//        std::vector<std::vector<point_heading_t<double>> get_drones_paths_wi

        /*!
         * Get path as a sequence of points to be visited from the sequence of targets to be visited
         * @param targets Sequence of targets to be visited by drone
         * @return Sequence of points to be visited by drone
         */
        std::vector<point_t> get_path_from_targets(const std::vector<Target> &targets) const;

        /*!
         * Add Retrieve a path with added heading along each sweep pattern
         * @param targets Targets of the solution in the order of visiting
         * @param unique_alt_id Id for unique altitude during changing the sub-polygon for collision avoidance
         * @return Path with the right heading
         */
        std::vector<point_heading_t < double>> path_with_heading(
        const std::vector<Target> &targets
        ) const;

        /*!
         * Apply step 1: Random Shift
         * @param solution solution to modify
         */
        void get_g1_solution(_instance_solution_t &solution) const;

        /*!
         * Apply step 2: Best shift
         * @param solution solution to modify
         */
        void get_g2_solution(_instance_solution_t &solution) const;

        /*!
         * Apply step 3: Best swap
         * @param solution solution to modify
         */
        void get_g3_solution(_instance_solution_t &solution) const;

        /*!
         * Apply step 4: Direction change
         * @param solution solution to modify
         */
        void get_g4_solution(_instance_solution_t &solution) const;

        /*!
         * Given two nodes, select the best replacements from the same sets for them
         * @param solution Solution to modify
         * @param uav1 Index of path of the first node
         * @param path_index_1 Index inside the path of the first node
         * @param uav2 Index of path of the second node
         * @param path_index_2 Index inside the path of the second node
         */
        void find_best_targets_for_position(_instance_solution_t &solution, size_t uav1, size_t path_index_1,
                                            size_t uav2, size_t path_index_2) const;


    };
}


#endif //THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H
