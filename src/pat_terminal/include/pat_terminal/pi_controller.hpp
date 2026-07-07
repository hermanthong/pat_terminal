#pragma once

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
    return 0.0;
  }

private:
  PIParams params_;
};

}  // namespace pat_terminal
