#ifndef TIME_CALCULATOR_HPP
#define TIME_CALCULATOR_HPP

#include "PathCostCalculator.hpp"
#include "custom_types.hpp" // For point_heading_t

struct time_calculator_config_t {
    double max_speed;                   // [m/s] (corresponds to v_r in the energy model)
    double max_acceleration;            // [m/s^2] (maximum horizontal acceleration)
    double allowed_path_deviation;      // [m] (allowed deviation in turns for speed calculation)
};

class TimeCalculator : public PathCostCalculator {
private:
    time_calculator_config_t m_config;
    
    // Helper struct to represent turning properties.
    struct turning_properties_time_t {
        double v_before; // Speed before the turn
        double a_before; // Acceleration/deceleration before the turn
        double v_after;  // Speed after the turn
        double a_after;  // Acceleration after the turn
        double time;     // Time spent in the turn
        double d_vym;    // Geometric displacement
    };

    double max_horizontal_speed;       // [m/s] (corresponds to v_r in the energy model)
    double max_vertical_speed;         // [m/s] (maximum ascent/descent speed)
    double horizontal_acceleration;     // [m/s^2] (average horizontal acceleration)
    double vertical_acceleration;       // [m/s^2] (average vertical acceleration)

    turning_properties_time_t calculate_turning_properties(double angle, double v_r_effective) const;

    double calculate_segment_time_generic(double v_in, double a_in, double v_out, double a_out, double s, double v_max) const;
    double calculate_short_segment_time(double v_in, double a_in, double v_out, double a_out, double s, double v_max) const;

public:
    explicit TimeCalculator(const time_calculator_config_t& config);
    ~TimeCalculator() override = default;

    double calculate_path_cost(const std::vector<point_heading_t<double>>& path) const override;
    
    double calculate_segment_cost(double v_in, double a_in, double v_out, double a_out, double s) const override;

    // Methods for calculating decoupled 1D/2D movements
    double calculate_horizontal_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const;
    double calculate_vertical_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const;

    double get_max_speed() const override { return max_horizontal_speed; }
    double get_acceleration() const override { return horizontal_acceleration; }
};

#endif // TIME_CALCULATOR_HPP