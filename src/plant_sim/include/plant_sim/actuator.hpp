#pragma once

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
   * @brief Advance the model by one time step.
   * @return the actual position after the step in rad
   */
  double step(double command, double dt) {
    return 0.0;
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
