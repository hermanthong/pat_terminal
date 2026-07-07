#include "rclcpp/rclcpp.hpp"
#include "pat_interfaces/msg/axis_command.hpp"
#include "pat_interfaces/msg/axis_state.hpp"
#include "pat_interfaces/msg/mode_state.hpp"
#include "pat_terminal/low_pass.hpp"

using namespace std::chrono_literals;
using pat_interfaces::msg::AxisCommand;
using pat_interfaces::msg::AxisState;
using pat_interfaces::msg::ModeState;
using pat_terminal::LowPass;

/**
 * @brief One axis of the coarse loop
 */
struct CoarseAxis {
  LowPass offload;
  double bearing;              // [rad] commanded bearing
  double command{0.0};         // [rad] current gimbal command
  double fsm_deflection{0.0};  // [rad] latest FSM state, the offload input
};

/**
 * @brief The 100 Hz coarse loop: slews the gimbal to the commanded bearing in
 * ACQUIRE and offloads the FSM's average deflection in LOCK. Holds otherwise
 */
class CoarseController : public rclcpp::Node {
public:
  CoarseController()
  : Node("coarse_controller"),
    offload_tau_(declare_parameter("offload_tau", 1.0)),
    azimuth_{LowPass(offload_tau_), declare_parameter("bearing_azimuth", 0.1)},
    elevation_{LowPass(offload_tau_), declare_parameter("bearing_elevation", 0.05)} {
    gimbal_cmd_pub_ = create_publisher<AxisCommand>("gimbal_cmd", 10);

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
      azimuth_.command = azimuth_.bearing;
      elevation_.command = elevation_.bearing;
    
  }

  /**
   * @brief One 100 Hz cycle: in LOCK the gimbal steers toward wherever the
   * FSM is straining, so the FSM re-centers. fsm -> 0 is the equilibrium
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
  CoarseAxis azimuth_;
  CoarseAxis elevation_;
  uint8_t mode_{ModeState::IDLE};
  rclcpp::Time last_update_time_;

  rclcpp::Publisher<AxisCommand>::SharedPtr gimbal_cmd_pub_;
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
