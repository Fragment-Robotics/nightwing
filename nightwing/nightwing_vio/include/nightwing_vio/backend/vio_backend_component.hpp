#ifndef NIGHTWING_VIO_BACKEND_VIO_BACKEND_COMPONENT_HPP
#define NIGHTWING_VIO_BACKEND_VIO_BACKEND_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "nightwing_vio/backend/factor_graph_manager.hpp"

namespace nightwing {

namespace vio {

class VIOBackendComponent : public rclcpp::Node {
  public:
    explicit VIOBackendComponent(const rclcpp::NodeOptions& options);

  private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

    // Parameters to fill in
    struct Params {
        uint time_diff_s_;
        std::string directory_path_;
    };
    Params params_;

    // Tracking variables
    uint index_ = 0;
    rclcpp::Time last_time_seen_;

    // Backend
    FactorGraphManager factor_graph_manager_;
};

} // namespace vio

} // namespace nightwing

#endif