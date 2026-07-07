#include <gtest/gtest.h>
#include "pat_terminal/pi_controller.hpp"

using pat_terminal::PIController;
using pat_terminal::PIParams;

TEST(PIController, POnlyController) {
  PIController pi({.kp = 0.5, .ki = 0.0, .output_limit = 1e-3});
  double result = pi.update(200e-6, 0.001);
  // command = kp * error = 0.5 * 200 urad = 100 urad
  EXPECT_NEAR(result, 100e-6, 1e-12);
}

TEST(PIController, IOnlyController) {
  PIController pi({.kp = 0.0, .ki = 2.0, .output_limit = 1e-3});
  pi.update(100e-6, 0.001);
  pi.update(100e-6, 0.001);
  double result = pi.update(100e-6, 0.001);
  // command = ki * integral of error: 2.0 * (3 x 100 urad x 1 ms) = 0.6 urad
  EXPECT_NEAR(result, 6e-7, 1e-15);
}

TEST(PIController, CommandSaturatesAtOutputLimit) {
  PIController pi({.kp = 1.0, .ki = 0.0, .output_limit = 1e-3});
  double result1 = pi.update(2e-3, 0.001);
  EXPECT_NEAR(result1, 1e-3, 1e-12);
  double result2 = pi.update(-2e-3, 0.001);
  EXPECT_NEAR(result2, -1e-3, 1e-12);
}

TEST(PIController, IntegratorDoesNotWindUpPastSaturation) {
  PIController pi({.kp = 0.0, .ki = 1.0, .output_limit = 1e-3});
  for (int i = 0; i < 100; ++i) {
    // huge error so the output should saturate
    pi.update(1.0, 0.001);
  }
  double result1 = pi.update(-1.0, 0.001);
  EXPECT_NEAR(result1, 0.0, 1e-12);

  // the error persists, so the integrator legitimately drives the other way,
  // and the negative side is clamped at the limit just the same
  double result2 = pi.update(-1.0, 0.001);
  EXPECT_NEAR(result2, -1e-3, 1e-12);
}

TEST(PIController, ResetClearsTheIntegrator) {
  PIController pi({.kp = 0.0, .ki = 2.0, .output_limit = 1e-3});
  pi.update(100e-6, 0.001);
  pi.reset();

  double result = pi.update(0.0, 0.001);
  EXPECT_NEAR(result, 0.0, 1e-15);
}
