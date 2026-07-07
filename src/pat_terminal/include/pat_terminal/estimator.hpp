#pragma once

namespace pat_terminal {

// Complementary filter for one axis of pointing error.
// theta_k = alpha * (theta_{k-1} + omega_k * dt) + (1 - alpha) * a_k
class Estimator {
public:
  explicit Estimator(double alpha)
  : alpha_(alpha) {}

  /**
  * @brief Update estimator with IMU data: theta += rate * dt
  */
  void propagate(double rate, double dt) {theta_ += rate * dt;}

  /**
  * @brief Update estimator with camera data: theta = alpha * theta + (1 - alpha) * measurement
  */
  void correct(double measurement) {theta_ = alpha_ * theta_ + (1.0 - alpha_) * measurement;}

  /**
  * @brief Seed the estimate, for HANDOFF entry with the first valid measurement
  */
  void reset(double initial) {theta_ = initial;}

  /**
  * @return [rad] the current estimate of the pointing error
  */
  double estimate() const {return theta_;}

private:
  double alpha_;
  double theta_{0.0};
};

}  // namespace pat_terminal
