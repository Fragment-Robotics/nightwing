#include "nightwing_data/rgbd_sync_store.hpp"


namespace nightwing {

namespace data {

RGBDSyncStore::RGBDSyncStore(const rclcpp::NodeOptions& options) 
: Node("RGBDSyncStore") {
    // Individual callbacks
    color_sub_.subscribe(this, "/color/image_uncompressed");
    depth_sub_.subscribe(this, "/depth/image_uncompressed");

    // Setup syncer
    sync_ = std::make_shared<Sync>(MySyncPolicy(10), color_sub_, depth_sub_);
    sync_->registerCallback(&RGBDSyncStore::callback, this);

    // Fetch ros params
    this->declare_parameter("time_diff_s", 2);
    this->declare_parameter("directory_path", "/data/extracted_images/");
    this->get_parameter("time_diff_s", params_.time_diff_s_);
    this->get_parameter("directory_path", params_.directory_path_);

    RCLCPP_INFO(this->get_logger(), "RGBDSyncStore Node started with parameters:");
    RCLCPP_INFO(this->get_logger(), "  time_diff_s: %u", params_.time_diff_s_);
    RCLCPP_INFO(this->get_logger(), "  directory_path: %s", params_.directory_path_.c_str());
}

void RGBDSyncStore::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                             const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    // Convert color image to cv
    cv_bridge::CvImagePtr cv_ptr_color;
    try {
        cv_ptr_color = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Convert depth image to cv. NOTE: Depth images are color encoded to
    // leverage H264 encoding
    cv_bridge::CvImagePtr cv_ptr_depth;
    try {
        cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Check for timing difference between message timestamps and last seen
    rclcpp::Time current_time = color_msg->header.stamp;
    double time_diff;
    if (index_ == 0) {
        time_diff = params_.time_diff_s_ + 1;
    } else {
        time_diff = (current_time - last_time_seen_).seconds();
    }

    if (time_diff > params_.time_diff_s_) {
        // Save images to directory using current index in name
        std::string color_img_name = params_.directory_path_ + "color_" + std::to_string(index_) + ".png";
        std::string depth_img_name = params_.directory_path_ + "depth_" + std::to_string(index_) + ".png";

        cv::imwrite(color_img_name, cv_ptr_color->image);
        cv::imwrite(depth_img_name, cv_ptr_depth->image);

        // Update last seen time
        last_time_seen_ = current_time;
        index_ += 1;
    }

}

} // namespace data

} // namespace nightwing

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nightwing::data::RGBDSyncStore)

