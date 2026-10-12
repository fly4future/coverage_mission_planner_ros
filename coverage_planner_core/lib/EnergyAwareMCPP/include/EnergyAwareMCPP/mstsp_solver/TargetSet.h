//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_TARGETSET_H
#define THESIS_TRAJECTORY_GENERATOR_TARGETSET_H

#include "utils.hpp"
#include <vector>
#include "Target.h"
#include <cmath>
#include "MapPolygon.hpp"
#include "PathCostCalculator.hpp"

namespace mstsp_solver {

    struct TargetSet {
        size_t index;
        MapPolygon polygon;
        std::vector<Target> targets;
        const std::vector<std::shared_ptr<PathCostCalculator>>* cost_calculators;
        double sweeping_step;
        double m_wall_distance;

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step, double wall_distance,
                  const std::vector<std::shared_ptr<PathCostCalculator>>* cost_calculators) :
                TargetSet(index, polygon, sweeping_step, wall_distance, cost_calculators, std::vector<double>{0, M_PI}) {};

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step, double wall_distance,
                  const std::vector<std::shared_ptr<PathCostCalculator>>* cost_calculators, const std::vector<double> &rotation_angles);


        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step, double wall_distance,
                  const std::vector<std::shared_ptr<PathCostCalculator>>* cost_calculators, size_t number_of_edges_rotations);

    private:
        /*!
         * Delete all the stored nodes and add new ones, with rotation angle of each as angles
         * @param angles sweeping angles of inserted nodes
         */
        void set_rotation_angles(const std::vector<double>& angles);

        /*!
         * Generate 2 nodes corresponding to the given rotation angle and store it
         * @param angle Rotation angle for sweeping
         * @param up If the first sweep should go up
         */
        void add_one_rotation_angle(double angle, bool up);
    };
}

#endif //THESIS_TRAJECTORY_GENERATOR_TARGETSET_H
