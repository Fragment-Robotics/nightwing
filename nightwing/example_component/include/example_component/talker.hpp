#ifndef EXAMPLE_COMPONENT_TALKER_HH
#define EXAMPLE_COMPONENT_TALKER_HH

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


namespace nightwing {

class Talker : public rclcpp::Node {
  public:
    explicit Talker(const rclcpp::NodeOptions& options);

  protected:
    void timer_callback();

  private:
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

};

} // namespace nightwing

#endif