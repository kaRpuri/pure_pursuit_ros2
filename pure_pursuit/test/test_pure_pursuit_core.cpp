#include <gtest/gtest.h>

#include "pure_pursuit/pure_pursuit_core.hpp"

namespace
{

// Defaults used in simulation, see config/pure_pursuit_sim.yaml.
constexpr double kLookahead = 1.72;
constexpr double kSteeringGain = 0.435;
const pure_pursuit::SpeedProfile kProfile{0.4, 4.5, 1.5};

// Car at the origin facing along +x.
Eigen::Matrix3d identityHeading()
{
  return pure_pursuit::worldToVehicleRotation(Eigen::Quaterniond::Identity());
}

double steeringForGoal(const Eigen::Vector2d & goal)
{
  const Eigen::Vector2d local = pure_pursuit::goalPointInVehicleFrame(
    identityHeading(), Eigen::Vector2d(0.0, 0.0), goal);
  return pure_pursuit::steeringAngle(
    kSteeringGain, pure_pursuit::arcCurvature(local.y(), kLookahead));
}

TEST(PurePursuitCore, StraightAheadGoalGivesZeroSteering)
{
  EXPECT_DOUBLE_EQ(steeringForGoal(Eigen::Vector2d(kLookahead, 0.0)), 0.0);
}

TEST(PurePursuitCore, GoalToTheLeftGivesPositiveSteering)
{
  EXPECT_GT(steeringForGoal(Eigen::Vector2d(1.5, 0.8)), 0.0);
}

TEST(PurePursuitCore, GoalToTheRightGivesNegativeSteering)
{
  EXPECT_LT(steeringForGoal(Eigen::Vector2d(1.5, -0.8)), 0.0);
}

TEST(PurePursuitCore, HeadingIsAppliedToTheGoalPoint)
{
  // Car rotated 90 degrees counter clockwise. A goal straight ahead in the
  // world +x direction then sits to the right of the car.
  const Eigen::Quaterniond yaw90(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector2d local = pure_pursuit::goalPointInVehicleFrame(
    pure_pursuit::worldToVehicleRotation(yaw90),
    Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(2.0, 0.0));
  EXPECT_NEAR(local.x(), 0.0, 1e-9);
  EXPECT_NEAR(local.y(), -2.0, 1e-9);
}

TEST(PurePursuitCore, SpeedStaysWithinTheCommandedRange)
{
  // The lower bound is the 2.0 clamp in speedFromSteering. The upper bound is
  // 5.0 rather than max_speed because of the "if (speed > 4.3) speed = 5.0"
  // clamp that the tuned controller relies on.
  for (double steering = -1.0; steering <= 1.0; steering += 0.01) {
    const double speed = pure_pursuit::speedFromSteering(steering, kProfile);
    EXPECT_GE(speed, 2.0);
    EXPECT_LE(speed, 5.0);
  }
}

TEST(PurePursuitCore, LargerSteeringNeverIncreasesSpeedOnceBelowTheUpperClamp)
{
  // Past the upper clamp the profile is monotonically decreasing.
  const double a = pure_pursuit::speedFromSteering(0.1, kProfile);
  const double b = pure_pursuit::speedFromSteering(0.3, kProfile);
  EXPECT_GE(a, b);
}

}  // namespace
