#include <gtest/gtest.h>
#include "pat_terminal/pi_controller.hpp"

using pat_terminal::PIController;
using pat_terminal::PIParams;

TEST(PIController, POnlyController) {
  PIController pi({.kp = 0.5, .ki = 0.0, .output_limit = 1e-3});
  EXPECT_NEAR(pi.update(200e-6, 0.001), 100e-6, 1e-12);
}
