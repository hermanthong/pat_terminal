#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "pat_interfaces/msg/axis_command.hpp"
#include "pat_interfaces/msg/axis_state.hpp"
#include "pat_interfaces/msg/mode_state.hpp"
#include "pat_interfaces/msg/position_error.hpp"
#include "pat_terminal/low_pass.hpp"

using namespace std::chrono_literals;
using pat_interfaces::msg::AxisCommand;
using pat_interfaces::msg::AxisState;
using pat_interfaces::msg::ModeState;
using pat_interfaces::msg::PositionError;
using pat_terminal::LowPass;

/**
 * @brief One axis of the coarse loop
 */
struct CoarseAxis {
  LowPass offload;
  double command{0.0};         // [rad] current gimbal command
  double fsm_deflection{0.0};  // [rad] latest FSM state, the offload input
};

/**
 * @brief The 100 Hz coarse loop: in ACQUIRE the gimbal steers to center the
 * spot using the camera error, in LOCK it offloads the FSM's average
 * deflection. Holds otherwise
 */
class CoarseController : public rclcpp::Node {
public:
  CoarseController()
  : Node("coarse_controller"),
    offload_tau_(declare_parameter("offload_tau", 5.0)),
    acquire_gain_(declare_parameter("acquire_gain", 0.3)),
    acquire_step_limit_(declare_parameter("acquire_step_limit", 5e-3)),
    steering_deadband_(declare_parameter("steering_deadband", 100e-6)),
    azimuth_{LowPass(offload_tau_)},
    elevation_{LowPass(offload_tau_)} {
    gimbal_cmd_pub_ = create_publisher<AxisCommand>("gimbal_cmd", 10);

    // the camera error steers the gimbal toward the spot until LOCK, where
    // the offload takes over as the gimbal's steering source
    position_error_sub_ = create_subscription<PositionError>(
      "position_error", 10, [this](const PositionError & msg) {
        const bool steering = mode_ == ModeState::ACQUIRE || mode_ == ModeState::HANDOFF;
        if (!steering || !msg.valid) {
          return;
        }
        azimuth_.command += steering_step(msg.error_azimuth);
        elevation_.command += steering_step(msg.error_elevation);
      });

    fsm_state_sub_ = create_subscription<AxisState>(
      "fsm_state", 10, [this](const AxisState & msg) {
        azimuth_.fsm_deflection = msg.azimuth;
        elevation_.fsm_deflection = msg.elevation;
      });

    mode_sub_ = create_subscription<ModeState>(
      "mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](const ModeState & msg) {mode_callback(msg);});

    last_update_time_ = now();
    update_timer_ = create_wall_timer(10ms, [this] {update();});
  }

private:
  /**
   * @brief One steering increment. Deadzone and clamping applied.
   * @return how far to move the gimbal command this camera frame in rad
   */
  double steering_step(double error) const {
    if (std::abs(error) < steering_deadband_) {
      return 0.0;
    }
    return std::clamp(acquire_gain_ * error, -acquire_step_limit_, acquire_step_limit_);
  }

  /**
   * @return true when the offload is running
   */
  bool active() const {
    return mode_ == ModeState::LOCK;
  }

  void mode_callback(const ModeState & msg) {
    if (msg.mode == mode_) {
      return;
    }
    mode_ = msg.mode;
    if (mode_ == ModeState::LOCK) {
      azimuth_.offload.reset(azimuth_.command);
      elevation_.offload.reset(elevation_.command);
    }
  }

  /**
   * @brief One 100 Hz cycle. Updates both axes' offload filters and command the gimbal
   * @note In LOCK, the gimbal steers to offload the FSM
   */
  void update() {
    const auto update_time = now();
    const double dt = (update_time - last_update_time_).seconds();
    last_update_time_ = update_time;
    if (dt <= 0.0) {
      return;
    }

    if (active()) {
      azimuth_.command = azimuth_.offload.update(
        azimuth_.command + azimuth_.fsm_deflection, dt);
      elevation_.command = elevation_.offload.update(
        elevation_.command + elevation_.fsm_deflection, dt);
    }

    AxisCommand msg;
    msg.header.stamp = update_time;
    msg.azimuth = azimuth_.command;
    msg.elevation = elevation_.command;
    gimbal_cmd_pub_->publish(msg);
  }

  double offload_tau_;
  double acquire_gain_;
  // maximum gimbal command change per camera frame. should be slightly lower than
  // the gimbal's physical rate limit to avoid overshoot oscillation.
  double acquire_step_limit_;
  // [rad] below this the gimbal holds still to prevent unnecessary oscillations at steady state.
  double steering_deadband_;
  CoarseAxis azimuth_;
  CoarseAxis elevation_;
  uint8_t mode_{ModeState::IDLE};
  rclcpp::Time last_update_time_;

  rclcpp::Publisher<AxisCommand>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Subscription<PositionError>::SharedPtr position_error_sub_;
  rclcpp::Subscription<AxisState>::SharedPtr fsm_state_sub_;
  rclcpp::Subscription<ModeState>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoarseController>());
  rclcpp::shutdown();
  return 0;
}
