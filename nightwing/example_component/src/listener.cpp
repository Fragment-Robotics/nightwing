#include <functional>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "example_component/listener.hpp"

using namespace std::chrono_literals;

namespace nightwing {

Listener::Listener(const rclcpp::NodeOptions& options)
: Node("listener", options) {
  // Create subscriber
  sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 1, std::bind(&Listener::message_callback, this, std::placeholders::_1));
}

void Listener::message_callback(const std_msgs::msg::String& msg) {
  RCLCPP_INFO(this->get_logger(), "Received msg %s", msg.data.c_str());
  std::flush(std::cout);
}

} // namespace nightwing

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with the class_loader
// This acts as a sort of entry point, allowing the component to be discoverable wieh its library
// is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(nightwing::Listener)