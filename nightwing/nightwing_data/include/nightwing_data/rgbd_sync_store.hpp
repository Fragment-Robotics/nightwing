#ifndef RGBD_SYNC_STORE_HH
#define RGBD_SYNC_STORE_HH

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"


namespace nightwing {

namespace data {

class RGBDSyncStore : public rclcpp::Node {
  public:
    explicit RGBDSyncStore(const rclcpp::NodeOptions& options);

  private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

    // Pub/Sub
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;

    // Sync functionality
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    // Parameters to fill in
    struct Params {
        uint time_diff_s_;
        std::string directory_path_;
    };
    Params params_;

    // Tracking variables
    uint index_ = 0;
    rclcpp::Time last_time_seen_;

};

} // namespace data

} // namespace nightwing

# endif



