#pragma once

namespace pat_terminal {

/**
 * @brief First-order low-pass filter for the gimbal offload
 */
class LowPass {
public:
  explicit LowPass(double tau)
  : tau_(tau) {}

  /**
   * @brief One filter step: y += (dt / tau) * (sample - y)
   * @return the filtered value after the step
   */
  double update(double sample, double dt) {
    value_ += (dt / tau_) * (sample - value_);
    return value_;
  }

  /**
   * @brief Seed the filter, for starting the offload from the current command
   * @param initial the initial value to seed the filter with
   */
  void reset(double initial) {value_ = initial;}

  /**
   * @return the current filtered value
   */
  double value() const {return value_;}

private:
  double tau_;
  double value_{0.0};
};

}  // namespace pat_terminal
