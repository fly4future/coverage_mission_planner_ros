#include "TimeCalculator.hpp"
#include "utils.hpp"
#include "EnergyCalculator.h" // For access to static helper methods like angle_between_points
#include <cmath>
#include <algorithm>
#include <stdexcept>

TimeCalculator::TimeCalculator(const time_calculator_config_t& config) : m_config(config) {
    max_horizontal_speed = m_config.max_speed;
    max_vertical_speed = m_config.max_speed;
    horizontal_acceleration = m_config.max_acceleration;
    vertical_acceleration = m_config.max_acceleration;
}

double TimeCalculator::calculate_segment_cost(double v_in, double a_in, double v_out, double a_out, double s) const {
    return calculate_horizontal_segment_time(v_in, a_in, v_out, a_out, s);
}

// Calculates the time to traverse a segment of length 's' using a trapezoidal velocity profile. This is a generic implementation for any 1D movement.
double TimeCalculator::calculate_segment_time_generic(double v_in, double a_in, double v_out, double a_out, double s, double v_max) const {
    if (s <= 1e-5) return 0.0;

    double t_acc = std::abs(v_max - v_in) / a_in;
    double s_acc = v_in * t_acc + 0.5 * a_in * std::pow(t_acc, 2);

    double t_dec = std::abs((v_max - v_out) / a_out);
    double s_dec = v_max * t_dec + 0.5 * a_out * std::pow(t_dec, 2);

    if (s_acc + s_dec <= s) { // Case 1: Max speed is reached.
        return t_acc + t_dec + (s - s_acc - s_dec) / v_max;
    } else { // Case 2: Segment is too short to reach max speed.
        return calculate_short_segment_time(v_in, a_in, v_out, a_out, s, v_max);
    }
}

// Calculates time for a horizontal segment by calling the generic implementation.
double TimeCalculator::calculate_horizontal_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const {
    return calculate_segment_time_generic(v_in, a_in, v_out, a_out, s, max_horizontal_speed);
}

// Calculates time for a vertical segment by calling the generic implementation.
double TimeCalculator::calculate_vertical_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const {
    return calculate_segment_time_generic(v_in, a_in, v_out, a_out, s, max_vertical_speed);
}

// Handles cases where the segment is too short to reach max speed.
double TimeCalculator::calculate_short_segment_time(double v_in, double a_in, double v_out, double a_out, double s, double v_max) const {
    double a_abs_out = std::abs(a_out);
    double v_sol = std::sqrt((v_in * v_in / a_in + v_out * v_out / a_abs_out + 2 * s) / (1.0 / a_in + 1.0 / a_abs_out));
    
    // If no real solution is found, fall back to a simple average speed calculation.
    if (std::isnan(v_sol) || v_sol < v_in || v_sol < v_out || v_sol > v_max) {
        double avg_speed = (v_in + v_out) / 2.0;
        if (avg_speed < 1e-3) return 0.0;
        return s / avg_speed;
    }
    return (v_sol - v_in) / a_in + (v_sol - v_out) / a_abs_out;
}

// Calculates turning properties based on physics from the original EnergyCalculator.
TimeCalculator::turning_properties_time_t TimeCalculator::calculate_turning_properties(double angle, double v_r_effective) const {
    angle = std::abs(angle);
    double a_h = m_config.max_acceleration;

    // For a 180-degree turn, assume the drone can maintain speed.
    if (std::abs(angle * 180.0 / M_PI - 180.0) < 1.0) {
        return {v_r_effective, -a_h, v_r_effective, a_h, 0.0, 0.0};
    }

    double phi = M_PI - angle;
    double phi_2 = phi / 2.0;
    double d_x = m_config.allowed_path_deviation;
    double a_x = a_h * std::cos(phi_2);
    double a_y = a_h * std::sin(phi_2);
    
    double dv_x = std::sqrt(2.0 * d_x * a_x);
    dv_x = std::min(dv_x, std::cos(M_PI_2 - phi) * v_r_effective / 2.0);

    double dv_y = std::tan(phi_2) * dv_x;
    double vy_m = dv_x / std::tan(phi_2);

    double v_in = vy_m + dv_y;
    double t_turn = 2.0 * dv_y / a_y; // Time spent executing the turn maneuver.

    return {v_in, -a_y, v_in, a_y, t_turn, vy_m};
}

// Main cost calculation for a full 3D path.
double TimeCalculator::calculate_path_cost(const std::vector<point_heading_t<double>>& path) const {
    if (path.size() < 2) return 0.0;

    double total_time = 0.0;

    // Filter out adjacent duplicate points to avoid division by zero.
    std::vector<point_heading_t<double>> filtered_path;
    filtered_path.push_back(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {
        double d = std::sqrt(std::pow(path[i].x - path[i-1].x, 2) + std::pow(path[i].y - path[i-1].y, 2) + std::pow(path[i].z - path[i-1].z, 2));
        if (d > 1e-5) {
            filtered_path.push_back(path[i]);
        }
    }

    if (filtered_path.size() < 2) return 0.0;

    // STEP A: Pre-calculate the properties of all 3D turns in the path.
    std::vector<turning_properties_time_t> turns;
    turns.push_back({0.0, horizontal_acceleration, 0.0, horizontal_acceleration, 0.0, 0.0}); // Start from rest
    for (size_t i = 1; i + 1 < filtered_path.size(); ++i) {
        // Calculate effective max speed for the turn based on the 3D geometry of incoming and outgoing segments
        auto calc_effective_v_max = [&](const point_heading_t<double>& p_start, const point_heading_t<double>& p_end) {
            double d_3d = std::sqrt(std::pow(p_end.x - p_start.x, 2) + std::pow(p_end.y - p_start.y, 2) + std::pow(p_end.z - p_start.z, 2));
            if (d_3d < 1e-6) return max_horizontal_speed;

            double d_horiz = std::sqrt(std::pow(p_end.x - p_start.x, 2) + std::pow(p_end.y - p_start.y, 2));
            double d_vert = std::abs(p_end.z - p_start.z);

            double ratio_horiz = d_horiz / d_3d;
            double ratio_vert = d_vert / d_3d;

            double v_lim_h = (ratio_horiz > 1e-6) ? max_horizontal_speed / ratio_horiz : std::numeric_limits<double>::max();
            double v_lim_v = (ratio_vert > 1e-6) ? max_vertical_speed / ratio_vert : std::numeric_limits<double>::max();

            return std::min(v_lim_h, v_lim_v);
        };

        double v_eff_in = calc_effective_v_max(filtered_path[i-1], filtered_path[i]);
        double v_eff_out = calc_effective_v_max(filtered_path[i], filtered_path[i+1]);
        double v_r_turn = std::min(v_eff_in, v_eff_out);

        // Calculate the angle in 3D space between the incoming and outgoing segments.
        double angle = EnergyCalculator::angle_between_points_3d(filtered_path[i-1], filtered_path[i], filtered_path[i+1]);
        auto turn_props = calculate_turning_properties(angle, v_r_turn);
        turns.push_back(turn_props);
    }

    turns.push_back({0.0, -horizontal_acceleration, 0.0, -horizontal_acceleration, 0.0, 0.0}); // Stop at the end

    // STEP B: Calculate the time for each segment, considering decoupled horizontal and vertical motion.
    for (size_t i = 0; i + 1 < filtered_path.size(); ++i) {
        const auto& p1 = filtered_path[i];
        const auto& p2 = filtered_path[i+1];

        double horiz_dist = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
        double vert_dist = std::abs(p2.z - p1.z);
        double s_3d = std::sqrt(horiz_dist * horiz_dist + vert_dist * vert_dist);

        double segment_horiz_time = 0.0;
        double segment_vert_time = 0.0;

        const auto& turn1 = turns[i];
        const auto& turn2 = turns[i+1];

        double ratio_horiz = (s_3d > 1e-6) ? horiz_dist / s_3d : 0.0;
        double ratio_vert = (s_3d > 1e-6) ? vert_dist / s_3d : 0.0;


        // 1. Calculate horizontal flight time for the segment.
        if (horiz_dist > 1e-5) {
            double v_after_comp = turn1.v_after * ratio_horiz;
            double d_vym_comp1 = turn1.d_vym * ratio_horiz;
            double v_before_comp = turn2.v_before * ratio_horiz;
            double d_vym_comp2 = turn2.d_vym * ratio_horiz;

            double t_acc_slow = (turn1.a_after > 1e-6) ? std::abs((v_after_comp - d_vym_comp1) / turn1.a_after) : 0.0;
            double s_acc_slow = d_vym_comp1 * t_acc_slow + 0.5 * turn1.a_after * std::pow(t_acc_slow, 2);
            double t_dec_slow = (turn2.a_before < -1e-6) ? std::abs((v_before_comp - d_vym_comp2) / turn2.a_before) : 0.0;
            double s_dec_slow = d_vym_comp2 * t_dec_slow + 0.5 * turn2.a_before * std::pow(t_dec_slow, 2);

            if (s_acc_slow + s_dec_slow < horiz_dist) {
                double slow_acc_time = t_acc_slow + t_dec_slow;
                double straight_time = calculate_horizontal_segment_time(v_after_comp, horizontal_acceleration, v_before_comp, -horizontal_acceleration, horiz_dist - s_acc_slow - s_dec_slow);
                segment_horiz_time = slow_acc_time + straight_time;
            } else {
                segment_horiz_time = calculate_short_segment_time(d_vym_comp1, turn1.a_after, d_vym_comp2, turn2.a_before, horiz_dist, max_horizontal_speed);
            }
        }

        // 2. Calculate vertical flight time for the segment.
        if (vert_dist > 1e-5) {
            double v_after_comp = turn1.v_after * ratio_vert;
            double d_vym_comp1 = turn1.d_vym * ratio_vert;
            double v_before_comp = turn2.v_before * ratio_vert;
            double d_vym_comp2 = turn2.d_vym * ratio_vert;

            double t_acc_slow = (turn1.a_after > 1e-6) ? std::abs((v_after_comp - d_vym_comp1) / turn1.a_after) : 0.0;
            double s_acc_slow = d_vym_comp1 * t_acc_slow + 0.5 * turn1.a_after * std::pow(t_acc_slow, 2);
            double t_dec_slow = (turn2.a_before < -1e-6) ? std::abs((v_before_comp - d_vym_comp2) / turn2.a_before) : 0.0;
            double s_dec_slow = d_vym_comp2 * t_dec_slow + 0.5 * turn2.a_before * std::pow(t_dec_slow, 2);

            if (s_acc_slow + s_dec_slow < vert_dist) {
                double slow_acc_time = t_acc_slow + t_dec_slow;
                double straight_time = calculate_vertical_segment_time(v_after_comp, vertical_acceleration, v_before_comp, -vertical_acceleration, vert_dist - s_acc_slow - s_dec_slow);
                segment_vert_time = slow_acc_time + straight_time;
            } else {
                segment_vert_time = calculate_short_segment_time(d_vym_comp1, turn1.a_after, d_vym_comp2, turn2.a_before, vert_dist, max_vertical_speed);
            }
        }

        // STEP C: Combine the times for the segment.
        // The total time for a 3D segment is determined by the slower of the two decoupled axes (the one that takes longer).
        // We also add the time spent physically maneuvering in the turn at the beginning of the segment (`turn1.time`).
        // This turn time is independent of the straight-line travel time.
        total_time += std::max(segment_horiz_time, segment_vert_time) + turn1.time;
    }

    return total_time;
}