#include <gtest/gtest.h>
#include "plant_sim/actuator.hpp"

using plant_sim::Actuator;
using plant_sim::ActuatorParams;

TEST(Actuator, ChasesTheCommandFirstOrder) {
  // limits far away, pure first-order lag with dt / tau = 0.1
  Actuator actuator({.tau = 0.1, .rate_limit = 1e9, .range_limit = 1e9});
  double result1 = actuator.step(1.0, 0.01);
  // x1 = 0 + 0.1 * (1 - 0) = 0.1
  EXPECT_NEAR(result1, 0.1, 1e-12);
  double result2 = actuator.step(1.0, 0.01);
  // x2 = 0.1 + 0.1 * (1 - 0.1) = 0.19
  EXPECT_NEAR(result2, 0.19, 1e-12);
}
