#include "rclcpp/rclcpp.hpp"
#include "pat_interfaces/msg/mode_state.hpp"

using pat_interfaces::msg::ModeState;

class ModeManager : public rclcpp::Node {
public:
  ModeManager()
  : Node("mode_manager")
  {
    mode_pub_ = create_publisher<ModeState>("mode", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_mode(); });
  }

private:
  void publish_mode() {
    ModeState msg;
    msg.header.stamp = now();
    msg.mode = ModeState::IDLE;
    mode_pub_->publish(msg);
  }

  rclcpp::Publisher<ModeState>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeManager>());
  rclcpp::shutdown();
  return 0;
}
