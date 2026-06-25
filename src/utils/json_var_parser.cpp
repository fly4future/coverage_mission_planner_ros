#include "mrs_coverage_planner/utils/json_var_parser.h"
#include <ros/ros.h>

namespace mrs_coverage_planner
{

namespace utils
{

template <typename T>
struct is_vector : std::false_type
{
};

template <typename U>
struct is_vector<std::vector<U>> : std::true_type
{
};

template <typename T>
constexpr bool is_vector_v = is_vector<T>::value;

// Updated convertFromJson function
template <typename T>
struct is_pair : std::false_type
{
};

template <typename U, typename V>
struct is_pair<std::pair<U, V>> : std::true_type
{
};

template <typename T>
constexpr bool is_pair_v = is_pair<T>::value;

template <typename T>
T convertFromJson(const json &j) {
  if constexpr (is_pair_v<T>) {
    using first_t  = typename T::first_type;
    using second_t = typename T::second_type;
    return {convertFromJson<first_t>(j.at("polygon")), convertFromJson<second_t>(j.at("max_altitude"))};
  } else if constexpr (is_vector_v<T>) {
    T result;
    result.reserve(j.size());
    for (const auto &item : j) {
      result.emplace_back(convertFromJson<typename T::value_type>(item));
    }
    return result;
  } else {
    return T(j);
  }
}

bool parseVar(const json &js, std::pair<std::string_view, parseable_t> &var) {
  const auto &var_name = var.first;

  if (!js.contains(var_name)) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[Var-parser]: JSON doesn't have the expected member \"" << var_name << "\".");
    return false;
  }

  auto &var_out = var.second;
  bool success = true;
  std::visit(
      [var_name, &js, &success](auto &&var_out) {
        using T = std::remove_pointer_t<std::decay_t<decltype(var_out)>>;
        try {
          *var_out = convertFromJson<T>(js.at(var_name));
        }
        catch (json::exception &e) {
          ROS_WARN_STREAM("[Var-parser]: Cannot parse member \"" << var_name << "\" (value: " << js.at(var_name) << ") as custom type: " << e.what());
          success = false;
        }
        catch (std::exception &e) {
          ROS_WARN_STREAM("[Var-parser]: Cannot parse member \"" << var_name << "\" - " << e.what());
          success = false;
        }
      },
      var_out);
  return success;
}

bool parseVars(const json &js, std::vector<std::pair<std::string_view, parseable_t>> &&vars) {
  // for (auto &var : vars) {
  //   if (!parseVar(js, var)) {
  //     ROS_WARN("parseVars: Could not load parameter %s", var.first);
  //     // return false;
  //     continue;
  //   }
  // }
  // return true;

  for (auto &var : vars)
    if (!parseVar(js, var))
      return false;
  return true;
}

} // namespace utils
} // namespace mrs_coverage_planner
