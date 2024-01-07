#ifndef EXAMPLE_COMPONENT_LISTENER_HH
#define EXAMPLE_COMPONENT_LISTENER_HH

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace nightwing {

class Listener : public rclcpp::Node {
  public:
    explicit Listener(const rclcpp::NodeOptions& options);
  
  private:
    void message_callback(const std_msgs::msg::String& msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

};

} // namespace nightwing

#endif