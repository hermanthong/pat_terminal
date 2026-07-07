#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "pat_interfaces/msg/mode_state.hpp"
#include "pat_interfaces/msg/position_error.hpp"
#include "pat_interfaces/srv/set_mode.hpp"
#include "pat_terminal/mode_logic.hpp"

using namespace std::chrono_literals;
using pat_interfaces::msg::ModeState;
using pat_interfaces::msg::PositionError;
using pat_interfaces::srv::SetMode;
using pat_terminal::Mode;
using pat_terminal::ModeLogic;
using pat_terminal::ModeParams;

class ModeManager : public rclcpp::Node {
public:
  ModeManager()
  : Node("mode_manager"),
    logic_(ModeParams{
        .lock_error_threshold = declare_parameter("lock_error_threshold", 50e-6),
        .lock_debounce_s = declare_parameter("lock_debounce_s", 0.200),
        .handoff_error_threshold = declare_parameter("handoff_error_threshold", 0.5e-3),
        .handoff_timeout_s = declare_parameter("handoff_timeout_s", 2.0),
        .coast_entry_s = declare_parameter("coast_entry_s", 0.100),
        .coast_timeout_s = declare_parameter("coast_timeout_s", 0.500),
      }) {
      current_mode_ = logic_.mode();
    // transient_local so a late joiner or a respawned node can learn the mode
    mode_pub_ = create_publisher<ModeState>("mode", rclcpp::QoS(1).reliable().transient_local());

    position_error_sub_ = create_subscription<PositionError>(
      "position_error", 10, [this](const PositionError & msg) {
        latest_error_ = msg;
        fresh_frame_ = true;
      });

    set_mode_service_ = create_service<SetMode>(
      "set_mode", [this](const SetMode::Request::SharedPtr request,
        SetMode::Response::SharedPtr response) {
        response->accepted = logic_.request(static_cast<Mode>(request->mode));
        publish_mode();
      });

    last_tick_time_ = now();
    tick_timer_ = create_wall_timer(16667us, [this] {update();}); // 60 Hz, same as camera
  }

private:
  /**
   * @brief Update cycle. Process camera data and advance the state machine.
   * @note runs at 60 Hz, same as the camera.
   */
  void update() {
    const auto tick_time = now();
    const double dt = (tick_time - last_tick_time_).seconds();
    last_tick_time_ = tick_time;

    logic_.tick({
      .dt = dt,
      .spot_valid = fresh_frame_ && latest_error_.valid,
      .error = std::hypot(latest_error_.error_azimuth, latest_error_.error_elevation),
    });
    fresh_frame_ = false;
    
    if (logic_.mode() != current_mode_) {
      RCLCPP_INFO(get_logger(), "mode: %s", pat_terminal::mode_name(logic_.mode()));
      current_mode_ = logic_.mode();
    }
    publish_mode();
  }

  void publish_mode() {
    ModeState msg;
    msg.header.stamp = now();
    msg.mode = static_cast<uint8_t>(logic_.mode());
    mode_pub_->publish(msg);
  }

  ModeLogic logic_;
  Mode current_mode_;
  PositionError latest_error_;
  bool fresh_frame_{false};
  rclcpp::Time last_tick_time_;

  rclcpp::Publisher<ModeState>::SharedPtr mode_pub_;
  rclcpp::Subscription<PositionError>::SharedPtr position_error_sub_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_service_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeManager>());
  rclcpp::shutdown();
  return 0;
}
