#pragma once

#include <cmath>

namespace plant_sim {

struct DisturbanceParams {
  double drift_rate;       // [rad/s] slow bearing drift
  double vib_amplitude_1;  // [rad] platform vibration, first component
  double vib_frequency_1;  // [Hz]
  double vib_amplitude_2;  // [rad] platform vibration, second component
  double vib_frequency_2;  // [Hz]
};

/**
 * @brief The true pointing disturbance as a deterministic function of time:
 * a slow bearing drift plus two fixed vibration sinusoids.
 */
class Disturbance {
public:
  explicit Disturbance(const DisturbanceParams & params)
  : params_(params) {}

  /**
   * @brief Get the disturbance angle at a given time.
   * @param t time since start of simulation in s
   * @return the disturbance angle at time t in rad
   */
  double angle_at(double t) const {
    return params_.drift_rate * t +
      params_.vib_amplitude_1 * std::sin(2.0 * M_PI * params_.vib_frequency_1 * t) +
      params_.vib_amplitude_2 * std::sin(2.0 * M_PI * params_.vib_frequency_2 * t);
  }

private:
  DisturbanceParams params_;
};

}  // namespace plant_sim
