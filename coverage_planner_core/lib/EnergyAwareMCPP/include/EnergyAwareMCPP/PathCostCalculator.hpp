#ifndef PATH_COST_CALCULATOR_HPP
#define PATH_COST_CALCULATOR_HPP

#include <vector>
#include "custom_types.hpp" // For point_heading_t

/**
 * @brief Abstract interface for calculating the cost of a path.
 * This allows the solver to be agnostic to the cost metric (e.g., energy, time).
 */
class PathCostCalculator {
public:
    virtual ~PathCostCalculator() = default;

    // Calculates the total cost (e.g., time in seconds or energy in Joules) for a given 3D path.
    virtual double calculate_path_cost(const std::vector<point_heading_t<double>>& path) const = 0;

    // Calculates the cost for a single straight segment of length 's' with given entry and exit velocities/accelerations.
    virtual double calculate_segment_cost(double v_in, double a_in, double v_out, double a_out, double s) const = 0;
    
    virtual double get_max_speed() const = 0;
    virtual double get_acceleration() const = 0;
};

#endif