#pragma once

#include <algorithm>

namespace plant_sim {

struct ActuatorParams {
  double tau;          // [s] first-order response time constant
  double rate_limit;   // [rad/s] max slew rate
  double range_limit;  // [rad] symmetric position clamp
};

/**
 * @brief Model of an actuator with simulated noise.
 * Gimbal: slow tau, tight rate limit, wide range. 
 * FSM: fast tau, wide rate, tight range.
 */
class Actuator {
public:
  explicit Actuator(const ActuatorParams & params)
  : params_(params) {}

  /**
   * @brief Advance the model by one time step. Rate and range limits are applied.
   * @param command the commanded position in rad
   * @param dt the time step in s
   * @return the actual position after the step in rad
   */
  double step(double command, double dt) {
    // dt / tau capped at 1. Any faster and it oscillates unstably.
    const double fraction = std::min(dt / params_.tau, 1.0);
    const double move = fraction * (command - position_);
    const double max_move = params_.rate_limit * dt;
    position_ += std::clamp(move, -max_move, max_move);
    position_ = std::clamp(position_, -params_.range_limit, params_.range_limit);
    return position_;
  }

  /**
   * @return the actual position in rad
   */
  double position() const {return position_;}

private:
  ActuatorParams params_;
  double position_{0.0};
};

}  // namespace plant_sim
