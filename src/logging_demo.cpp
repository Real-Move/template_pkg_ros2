// src/logging_demo.cpp
#include "rclcpp/rclcpp.hpp"

class LoggingDemo : public rclcpp::Node {
public:
  LoggingDemo() : Node("logging_demo") {
    // Parameter to emit extra debug messages at runtime
    this->declare_parameter<bool>("verbose_debug", false);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        const bool verbose = this->get_parameter("verbose_debug").as_bool();

        RCLCPP_DEBUG(this->get_logger(), "This is a DEBUG message (only if log level allows).");
        if (verbose) {
          RCLCPP_DEBUG(this->get_logger(), "Extra DEBUG because verbose_debug=true.");
        }
        RCLCPP_INFO(this->get_logger(),  "This is an INFO message.");
        RCLCPP_WARN(this->get_logger(),  "This is a WARN message.");
        RCLCPP_ERROR(this->get_logger(), "This is an ERROR message.");
        RCLCPP_FATAL(this->get_logger(), "This is a FATAL message.");
      });
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggingDemo>());
  rclcpp::shutdown();
  return 0;
}
