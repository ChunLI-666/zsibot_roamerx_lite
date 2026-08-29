#ifndef ROBOT_NAVIGO__MOTION_LIMITS_HPP_
#define ROBOT_NAVIGO__MOTION_LIMITS_HPP_

#include <cmath>

namespace robot_navigo::motion_limits
{

// Versioned against yz-robot-ctrl's SDK contract. Nav2 mirrors these values in
// navigo_params.yaml and test_navigation_contract.py rejects divergence.
constexpr double kNavigationMaxVx = 0.15;
constexpr double kNavigationMaxVy = 0.15;
constexpr double kNavigationMaxWz = 0.10;
constexpr double kExecutableMinVx = 0.05;
constexpr double kExecutableMinVy = 0.10;
constexpr double kExecutableMinWz = 0.02;

inline double executableOrZero(double value, double minimum)
{
  if (!std::isfinite(value) || (value != 0.0 && std::abs(value) < minimum)) {
    return 0.0;
  }
  return value;
}

}  // namespace robot_navigo::motion_limits

#endif  // ROBOT_NAVIGO__MOTION_LIMITS_HPP_
