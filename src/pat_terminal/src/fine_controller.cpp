#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pat_interfaces/msg/axis_command.hpp"
#include "pat_interfaces/msg/mode_state.hpp"
#include "pat_interfaces/msg/position_error.hpp"
#include "pat_terminal/estimator.hpp"
#include "pat_terminal/pi_controller.hpp"

using namespace std::chrono_literals;
using pat_interfaces::msg::AxisCommand;
using pat_interfaces::msg::ModeState;
using pat_interfaces::msg::PositionError;
using pat_terminal::Estimator;
using pat_terminal::PIController;
using pat_terminal::PIParams;

/**
 * @brief One axis of the fine loop.
 */
struct FineAxis {
  Estimator estimator;
  PIController pi;
  bool seeded{false};        // first valid frame after HANDOFF entry seeds the estimate
  double imu_rate{0.0};      // [rad/s] latest platform rate
  double command{0.0};       // [rad] current FSM deflection command
  double previous_command{0.0};
};

/**
 * @brief The 1 kHz fine loop: estimates the pointing error from IMU + camera
 * and drives the FSM to cancel it. Active in HANDOFF, LOCK and COAST,
 * centered otherwise
 */
class FineController : public rclcpp::Node {
public:
  FineController()
  : Node("fine_controller"),
    alpha_(declare_parameter("alpha", 0.95)),
    pi_params_{
      .kp = declare_parameter("kp", 0.5),
      .ki = declare_parameter("ki", 100.0),
      .output_limit = declare_parameter("fsm_range_limit", 1e-3)},
    azimuth_{Estimator(alpha_), PIController(pi_params_)},
    elevation_{Estimator(alpha_), PIController(pi_params_)} {
    fsm_cmd_pub_ = create_publisher<AxisCommand>("fsm_cmd", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", 10, [this](const sensor_msgs::msg::Imu & msg) {
        azimuth_.imu_rate = msg.angular_velocity.z;
        elevation_.imu_rate = msg.angular_velocity.y;
      });

    position_error_sub_ = create_subscription<PositionError>(
      "position_error", 10, [this](const PositionError & msg) {
        if (!msg.valid || !active()) {
          return;
        }
        correct_axis(azimuth_, msg.error_azimuth);
        correct_axis(elevation_, msg.error_elevation);
      });

    mode_sub_ = create_subscription<ModeState>(
      "mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](const ModeState & msg) {mode_callback(msg);});

    last_update_time_ = now();
    update_timer_ = create_wall_timer(1ms, [this] {update();});
  }

private:
  /**
   * @return true in the modes where the fine loop owns the FSM
   */
  bool active() const {
    return mode_ == ModeState::HANDOFF || mode_ == ModeState::LOCK || mode_ == ModeState::COAST;
  }

  void mode_callback(const ModeState & msg) {
    const bool was_active = active();
    mode_ = msg.mode;
    if (active() && !was_active) {
      azimuth_.seeded = false;
      elevation_.seeded = false;
      azimuth_.pi.reset();
      elevation_.pi.reset();
    }
  }

  /**
   * @brief A valid camera frame: the first one after HANDOFF entry seeds the
   * estimate, later ones blend in at weight 1 - alpha
   */
  void correct_axis(FineAxis & axis, double measurement) {
    if (axis.seeded) {
      axis.estimator.correct(measurement);
    } else {
      axis.estimator.reset(measurement);
      axis.seeded = true;
    }
  }

  /**
   * @brief One 1 kHz cycle. Propagates both axes' estimates and command the FSM
   * @note In HANDOFF, LOCK and COAST, the FSM is commanded to work against the estimated pointing error.
   */
  void update() {
    const auto update_time = now();
    const double dt = (update_time - last_update_time_).seconds();
    last_update_time_ = update_time;
    if (dt <= 0.0) {
      return;
    }

    step_axis(azimuth_, dt);
    step_axis(elevation_, dt);

    AxisCommand msg;
    msg.header.stamp = update_time;
    msg.azimuth = azimuth_.command;
    msg.elevation = elevation_.command;
    fsm_cmd_pub_->publish(msg);
  }

  /**
   * @brief Propgate the estimator with IMU data
   */
  void step_axis(FineAxis & axis, double dt) {
    const double fsm_self_rate = (axis.command - axis.previous_command) / dt;
    axis.estimator.propagate(axis.imu_rate - fsm_self_rate, dt);
    axis.previous_command = axis.command;
    axis.command = active() ? axis.pi.update(axis.estimator.estimate(), dt) : 0.0;
  }

  double alpha_;
  PIParams pi_params_;
  FineAxis azimuth_;
  FineAxis elevation_;
  uint8_t mode_{ModeState::IDLE};
  rclcpp::Time last_update_time_;

  rclcpp::Publisher<AxisCommand>::SharedPtr fsm_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<PositionError>::SharedPtr position_error_sub_;
  rclcpp::Subscription<ModeState>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FineController>());
  rclcpp::shutdown();
  return 0;
}
