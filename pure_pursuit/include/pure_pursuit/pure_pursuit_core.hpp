// Pure math used by the pure pursuit controller.
//
// These functions hold no ROS or node state so they can be unit tested on
// their own. The node is responsible for finding the goal waypoint and for
// publishing; everything below is a plain computation.

#ifndef PURE_PURSUIT__PURE_PURSUIT_CORE_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_CORE_HPP_

#include <cmath>

#include "Eigen/Dense"

namespace pure_pursuit
{

// Limits used to scale speed with steering effort.
struct SpeedProfile
{
  double max_steering;
  double max_speed;
  double min_speed;
};

// Rotation that maps a world frame vector into the vehicle frame.
inline Eigen::Matrix3d worldToVehicleRotation(const Eigen::Quaterniond & orientation)
{
  return orientation.normalized().toRotationMatrix().transpose();
}

// Goal point expressed in the vehicle frame. x is forward, y is left.
inline Eigen::Vector2d goalPointInVehicleFrame(
  const Eigen::Matrix3d & world_to_vehicle,
  const Eigen::Vector2d & car_position,
  const Eigen::Vector2d & goal_position)
{
  Eigen::Vector3d offset(
    goal_position.x() - car_position.x(),
    goal_position.y() - car_position.y(),
    0.0);
  Eigen::Vector3d rotated = world_to_vehicle * offset;
  return Eigen::Vector2d(rotated.x(), rotated.y());
}

// Curvature of the arc from the vehicle to the goal point: 2 * y / L^2.
inline double arcCurvature(double goal_y_in_vehicle_frame, double lookahead_distance)
{
  return 2.0 * goal_y_in_vehicle_frame / (lookahead_distance * lookahead_distance);
}

// Steering angle in radians. Positive turns left.
inline double steeringAngle(double steering_gain, double curvature)
{
  return steering_gain * curvature;
}

// Speed drops linearly as the steering angle approaches max_steering.
//
// The two literal clamps below reproduce the controller that was tuned on the
// car. Note that the upper clamp raises speed to 5.0, so the returned value
// can exceed profile.max_speed. See the README for the reasoning.
inline double speedFromSteering(double steering_angle, const SpeedProfile & profile)
{
  double steer_fraction = std::abs(steering_angle) / profile.max_steering;
  double speed = profile.max_speed - (profile.max_speed - profile.min_speed) * steer_fraction;
  if (speed > 4.3) {speed = 5.0;}
  if (speed < 2.0) {speed = 2.0;}
  return speed;
}

}  // namespace pure_pursuit

#endif  // PURE_PURSUIT__PURE_PURSUIT_CORE_HPP_
