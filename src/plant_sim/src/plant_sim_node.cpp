#include <cmath>
#include <deque>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "pat_interfaces/msg/axis_command.hpp"
#include "pat_interfaces/msg/axis_state.hpp"
#include "pat_interfaces/msg/position_error.hpp"
#include "plant_sim/actuator.hpp"
#include "plant_sim/disturbance.hpp"

using namespace std::chrono_literals;
using pat_interfaces::msg::AxisCommand;
using pat_interfaces::msg::AxisState;
using pat_interfaces::msg::PositionError;
using plant_sim::Actuator;
using plant_sim::ActuatorParams;
using plant_sim::Disturbance;
using plant_sim::DisturbanceParams;

/**
 * @brief One axis of the simulated world
 * a slow bearing drift plus two fixed vibration sinusoids.
 */
struct Axis {
  Actuator gimbal;
  Actuator fsm;
  Disturbance disturbance;
  double bearing_offset;    // [rad] where the counterpart actually is
  double true_error{0.0};   // [rad] disturbance + offset - gimbal - fsm
};

class PlantSimNode : public rclcpp::Node {
public:
  PlantSimNode()
  : Node("plant_sim") {
    const ActuatorParams gimbal_params{
      .tau = declare_parameter("gimbal_tau", 0.032),
      .rate_limit = declare_parameter("gimbal_rate_limit", 0.35),
      .range_limit = declare_parameter("gimbal_range_limit", 1.57),
    };
    const ActuatorParams fsm_params{
      .tau = declare_parameter("fsm_tau", 0.00016),
      .rate_limit = declare_parameter("fsm_rate_limit", 10.0),
      .range_limit = declare_parameter("fsm_range_limit", 1e-3),
    };
    const DisturbanceParams disturbance_params{
      .drift_rate = declare_parameter("drift_rate", 2e-6),
      .vib_amplitude_1 = declare_parameter("vib_amplitude_1", 50e-6),
      .vib_frequency_1 = declare_parameter("vib_frequency_1", 2.0),
      .vib_amplitude_2 = declare_parameter("vib_amplitude_2", 20e-6),
      .vib_frequency_2 = declare_parameter("vib_frequency_2", 7.0),
    };
    const double bearing_azimuth = declare_parameter("bearing_azimuth", 0.1);
    const double bearing_elevation = declare_parameter("bearing_elevation", 0.05);
    azimuth_ = Axis{Actuator(gimbal_params), Actuator(fsm_params),
      Disturbance(disturbance_params), bearing_azimuth};
    elevation_ = Axis{Actuator(gimbal_params), Actuator(fsm_params),
      Disturbance(disturbance_params), bearing_elevation};

    camera_latency_s_ = declare_parameter("camera_latency_s", 0.030);
    camera_noise_sigma_ = declare_parameter("camera_noise_sigma", 5e-6);
    camera_fov_ = declare_parameter("camera_fov", 5e-3);
    imu_noise_sigma_ = declare_parameter("imu_noise_sigma", 5e-6);
    imu_bias_ = declare_parameter("imu_bias", 2e-6);
    rng_.seed(declare_parameter("seed", 42));

    position_error_pub_ = create_publisher<PositionError>("position_error", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    fsm_state_pub_ = create_publisher<AxisState>("fsm_state", 10);
    true_error_pub_ = create_publisher<AxisState>("true_error", 10);

    fsm_cmd_sub_ = create_subscription<AxisCommand>(
      "fsm_cmd", 10, [this](const AxisCommand & msg) {fsm_cmd_ = msg;});
    gimbal_cmd_sub_ = create_subscription<AxisCommand>(
      "gimbal_cmd", 10, [this](const AxisCommand & msg) {gimbal_cmd_ = msg;});
    blockage_sub_ = create_subscription<std_msgs::msg::Bool>(
      "blockage", 10, [this](const std_msgs::msg::Bool & msg) {blocked_ = msg.data;});

    tick_timer_ = create_wall_timer(1ms, [this] {tick();});
    camera_timer_ = create_wall_timer(16667us, [this] {publish_camera();}); // 60fps
  }

private:
  /**
   * @brief Advance the world by one 1 kHz step and publish the fast topics
   */
  void tick() {
    const double dt = 0.001;
    time_s_ += dt;
    step_axis(azimuth_, gimbal_cmd_.azimuth, fsm_cmd_.azimuth, dt);
    step_axis(elevation_, gimbal_cmd_.elevation, fsm_cmd_.elevation, dt);

    error_history_.push_back({azimuth_.true_error, elevation_.true_error});
    const auto latency_ticks = static_cast<size_t>(camera_latency_s_ / dt);
    while (error_history_.size() > latency_ticks + 1) {
      error_history_.pop_front();
    }

    publish_imu(dt);
    publish_fsm_state();
    publish_true_error();
  }

  void step_axis(Axis & axis, double gimbal_cmd, double fsm_cmd, double dt) {
    axis.gimbal.step(gimbal_cmd, dt);
    axis.fsm.step(fsm_cmd, dt);
    axis.true_error = axis.disturbance.angle_at(time_s_) + axis.bearing_offset -
      axis.gimbal.position() - axis.fsm.position();
  }

  /**
   * @brief The IMU senses the platform body rate (the disturbance rate),
   * not the actuators' own motion, plus a constant bias and noise
   */
  void publish_imu(double dt) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    const double previous_t = time_s_ - dt;
    // z: azimuth rate
    // y: elevation rate
    msg.angular_velocity.z = imu_bias_ + gaussian(imu_noise_sigma_) +
      (azimuth_.disturbance.angle_at(time_s_) - azimuth_.disturbance.angle_at(previous_t)) / dt;
    msg.angular_velocity.y = imu_bias_ + gaussian(imu_noise_sigma_) +
      (elevation_.disturbance.angle_at(time_s_) - elevation_.disturbance.angle_at(previous_t)) / dt;
    imu_pub_->publish(msg);
  }

  void publish_fsm_state() {
    AxisState msg;
    msg.header.stamp = now();
    msg.azimuth = azimuth_.fsm.position();
    msg.elevation = elevation_.fsm.position();
    fsm_state_pub_->publish(msg);
  }

  // Ground truth for plots and the integration test. Does not exist in flight.
  void publish_true_error() {
    AxisState msg;
    msg.header.stamp = now();
    msg.azimuth = azimuth_.true_error;
    msg.elevation = elevation_.true_error;
    true_error_pub_->publish(msg);
  }

  /**
   * @brief The camera reports the truth as of one latency ago plus centroid
   * noise, stamped with the exposure time. Invalid when the spot is outside
   * the FOV or a blockage is scripted.
   */
  void publish_camera() {
    if (error_history_.empty()) {
      return;
    }
    const auto & exposure = error_history_.front();
    PositionError msg;
    msg.header.stamp = now() - rclcpp::Duration::from_seconds(camera_latency_s_);
    msg.error_azimuth = exposure[0] + gaussian(camera_noise_sigma_);
    msg.error_elevation = exposure[1] + gaussian(camera_noise_sigma_);
    msg.valid = !blocked_ &&
      std::abs(exposure[0]) < camera_fov_ && std::abs(exposure[1]) < camera_fov_;
    position_error_pub_->publish(msg);
  }

  double gaussian(double sigma) {
    return std::normal_distribution<double>(0.0, sigma)(rng_);
  }

  Axis azimuth_{Actuator({0, 0, 0}), Actuator({0, 0, 0}), Disturbance({0, 0, 0, 0, 0}), 0.0};
  Axis elevation_{Actuator({0, 0, 0}), Actuator({0, 0, 0}), Disturbance({0, 0, 0, 0, 0}), 0.0};
  double time_s_{0.0};
  std::deque<std::array<double, 2>> error_history_;
  AxisCommand fsm_cmd_;
  AxisCommand gimbal_cmd_;
  bool blocked_{false};

  double camera_latency_s_{};
  double camera_noise_sigma_{};
  double camera_fov_{};
  double imu_noise_sigma_{};
  double imu_bias_{};
  std::mt19937 rng_;

  rclcpp::Publisher<PositionError>::SharedPtr position_error_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<AxisState>::SharedPtr fsm_state_pub_;
  rclcpp::Publisher<AxisState>::SharedPtr true_error_pub_;
  rclcpp::Subscription<AxisCommand>::SharedPtr fsm_cmd_sub_;
  rclcpp::Subscription<AxisCommand>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blockage_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr camera_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantSimNode>());
  rclcpp::shutdown();
  return 0;
}
