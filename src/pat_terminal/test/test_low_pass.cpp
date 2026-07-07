#include <gtest/gtest.h>
#include "pat_terminal/low_pass.hpp"

using pat_terminal::LowPass;

TEST(LowPass, StepsTowardTheSample) {
  LowPass low_pass(1.0);
  double result1 = low_pass.update(1.0, 0.1);
  // result = 0 + 0.1 * (1 - 0) = 0.1
  EXPECT_NEAR(result1, 0.1, 1e-12);
  double result2 = low_pass.update(1.0, 0.1);
  // result = 0.1 + 0.1 * (1 - 0.1) = 0.19
  EXPECT_NEAR(result2, 0.19, 1e-12);
}
TEST(LowPass, ResetSeedsTheValue) {
  LowPass low_pass(1.0);
  low_pass.update(1.0, 0.1);
  low_pass.reset(0.5);
  EXPECT_NEAR(low_pass.value(), 0.5, 1e-12);
  double result = low_pass.update(1.0, 0.1);
  // result = 0.5 + 0.1 * (1 - 0.5) = 0.55
  EXPECT_NEAR(result, 0.55, 1e-12);
}

TEST(LowPass, ConvergesToConstantInput) {
  LowPass low_pass(1.0);
  for (int i = 0; i < 100; ++i) {
    low_pass.update(500e-6, 0.1);
  }
  EXPECT_NEAR(low_pass.value(), 500e-6, 1e-7);
}
