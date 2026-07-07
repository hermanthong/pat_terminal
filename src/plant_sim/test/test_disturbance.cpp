#include <gtest/gtest.h>
#include "plant_sim/disturbance.hpp"

using plant_sim::Disturbance;
using plant_sim::DisturbanceParams;

TEST(Disturbance, DriftPlusSinusoidsAtChosenTimes) {
  Disturbance disturbance({
    .drift_rate = 1e-6,
    .vib_amplitude_1 = 100e-6, .vib_frequency_1 = 1.0,
    .vib_amplitude_2 = 0.0, .vib_frequency_2 = 0.0,
  });
  // angle = 1e-6 * 0 + 100e-6 * sin(0) = 0
  EXPECT_NEAR(disturbance.angle_at(0.0), 0.0, 1e-12);
  // angle = 1e-6 * 0.25 + 100e-6 * sin(pi/2) = 100.25e-6
  EXPECT_NEAR(disturbance.angle_at(0.25), 100.25e-6, 1e-12);
  // angle = 1e-6 * 0.5 + 100e-6 * sin(pi) = 0.5e-6
  EXPECT_NEAR(disturbance.angle_at(0.5), 0.5e-6, 1e-12);
}
