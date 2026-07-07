#include <gtest/gtest.h>
#include "pat_terminal/estimator.hpp"

using pat_terminal::Estimator;

// theta_k = alpha * (theta_{k-1} + omega_k * dt) + (1 - alpha) * a_k

TEST(Estimator, PropagateIntegratesRateOverTime)
{
  Estimator estimator(0.95);
  estimator.propagate(0.002, 0.001);
  estimator.propagate(0.002, 0.001);
  estimator.propagate(0.002, 0.001);
  EXPECT_NEAR(estimator.estimate(), 6e-6, 1e-12);
}

TEST(Estimator, CorrectBlendsTowardMeasurement)
{
  Estimator estimator(0.95);
  estimator.propagate(0.1, 0.001);
  // 100 urad
  EXPECT_NEAR(estimator.estimate(), 0.0001, 1e-12);
  estimator.correct(0.0);
  // 95 urad
  EXPECT_NEAR(estimator.estimate(), 0.000095, 1e-12);
}
