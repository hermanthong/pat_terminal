#pragma once

namespace pat_terminal
{

// Complementary filter for one axis of pointing error.
// theta_k = alpha * (theta_{k-1} + omega_k * dt) + (1 - alpha) * a_k
class Estimator
{
public:
  explicit Estimator(double alpha)
  : alpha_(alpha) {}

  // For IMU data
  void propagate(double rate, double dt) {theta_ += rate * dt;}

  // For camera data 
  void correct(double measurement) {theta_ = alpha_ * theta_ + (1.0 - alpha_) * measurement;}

  double estimate() const {return theta_;}

private:
  double alpha_;
  double theta_{0.0};
};

}  // namespace pat_terminal
