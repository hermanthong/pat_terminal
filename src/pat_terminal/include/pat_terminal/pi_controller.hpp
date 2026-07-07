#pragma once

#include <algorithm>

namespace pat_terminal {

struct PIParams {
  double kp;            // Proportional gain
  double ki;            // [1/s] Integral gain
  double output_limit;  // [rad] symmetric clamp on the command (FSM range)
};

/**
 * @brief PI controller for each axis
 */
class PIController {
public:
  explicit PIController(const PIParams & params)
  : params_(params) {}

  /**
   * @brief One control step, run every control cycle
   * @return FSM deflection command in rad
   */
  double update(double error, double dt) {
    const double proportional_term = params_.kp * error;
    integral_term_ += params_.ki * error * dt;
    integral_term_ = std::clamp(integral_term_, -params_.output_limit, params_.output_limit);
    const double command = proportional_term + integral_term_;
    return std::clamp(command, -params_.output_limit, params_.output_limit);
  }

  /**
   * @brief Discard the accumulated integral, for mode transitions
   */
  void reset() {integral_term_ = 0.0;}

private:
  PIParams params_;
  double integral_term_{0.0};  // ki * accumulated error
};

}  // namespace pat_terminal
