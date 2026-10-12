//
// Created by mrs on 23.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_CUSTOM_TYPES_HPP
#define THESIS_TRAJECTORY_GENERATOR_CUSTOM_TYPES_HPP

#include <vector>

// Just convenient defines for some types
using point_t = std::pair<double, double>;
using segment_t = std::pair<point_t, point_t>;
using polygon_t = std::vector<std::pair<double, double>>;

/*!
 * Structure for representation of a no-fly zone with a maximum altitude
 */
struct HeightRestrictedNoFlyZone {
    polygon_t polygon;
    double max_altitude;
};

// Use a custom struct.
// The Reference3D from ROS is not used to make some modules completely independent of ROS
template<typename T=double>
struct point_heading_t {
    point_heading_t() = default;

    point_heading_t(T x, T y) : x{x}, y{y}, z{0}, heading{0} {};

    point_heading_t(T x, T y, T z) : x{x}, y{y}, z{z}, heading{0} {};

    explicit point_heading_t(std::pair<T, T> p) : x{p.first}, y{p.second}, z{0}, heading{0} {};
    T x;
    T y;
    T z;
    T heading;
};



#endif //THESIS_TRAJECTORY_GENERATOR_CUSTOM_TYPES_HPP
