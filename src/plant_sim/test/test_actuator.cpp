#include <gtest/gtest.h>
#include "plant_sim/actuator.hpp"

using plant_sim::Actuator;
using plant_sim::ActuatorParams;

TEST(Actuator, ChasesTheCommandFirstOrder) {
  // limits far away, pure first-order lag with dt / tau = 0.1
  Actuator actuator({.tau = 0.1, .rate_limit = 1e9, .range_limit = 1e9});
  double result1 = actuator.step(1.0, 0.01);
  // result = 0 + 0.1 * (1 - 0) = 0.1
  EXPECT_NEAR(result1, 0.1, 1e-12);
  double result2 = actuator.step(1.0, 0.01);
  // result = 0.1 + 0.1 * (1 - 0.1) = 0.19
  EXPECT_NEAR(result2, 0.19, 1e-12);
}

TEST(Actuator, RateLimitCapsTheSlew) {
  Actuator actuator({.tau = 0.01, .rate_limit = 0.5, .range_limit = 1e9});
  // capped at rate_limit * dt = 5 mrad per step
  double result1 = actuator.step(1.0, 0.01);
  EXPECT_NEAR(result1, 5e-3, 1e-12);

  double result2 = actuator.step(1.0, 0.01);
  EXPECT_NEAR(result2, 10e-3, 1e-12);
}

TEST(Actuator, PositionStopsAtRangeLimit) {
  Actuator actuator({.tau = 0.01, .rate_limit = 1e9, .range_limit = 1e-3});

  // command beyond the range
  double result1 = actuator.step(2e-3, 0.01);
  EXPECT_NEAR(result1, 1e-3, 1e-12);

  double result2 = actuator.step(-2e-3, 0.01);
  EXPECT_NEAR(result2, -1e-3, 1e-12);
}
