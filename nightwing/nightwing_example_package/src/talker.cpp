#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "example_component/talker.hpp"

using namespace std::chrono_literals;

namespace nightwing {

Talker::Talker (const rclcpp::NodeOptions& options)
: Node("talker", options), count_(0) {
  // Create publisher
  pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

  // Use a timer to schedule periodic message publishing
  timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
}

void Talker::timer_callback() {
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello World!" + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);

  // Publish message
  pub_->publish(std::move(msg));
}

} // namespace nightwing

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with the class_loader
// This acts as a sort of entry point, allowing the component to be discoverable wieh its library
// is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(nightwing::Talker)