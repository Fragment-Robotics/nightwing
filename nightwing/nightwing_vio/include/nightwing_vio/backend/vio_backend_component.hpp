#ifndef NIGHTWING_VIO_BACKEND_VIO_BACKEND_COMPONENT_HPP
#define NIGHTWING_VIO_BACKEND_VIO_BACKEND_COMPONENT_HPP

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Cal3_S2.h>

#include "rclcpp/rclcpp.hpp"


namespace nightwing {

namespace vio {

class VIOBackendComponent : public rclcpp::Node {
  public:
    explicit VIOBackendComponent(const rclcpp::NodeOptions& options);

    // Initialize the backend with the parameters
    bool initializeBackend();

    // Add new keyframe data and optimize the graph
    bool addStateandOptimize();
    
  private:

    // Factor graph manager
    std::shared_ptr<gtsam::NonlinearFactorGraph> graph_;
    std::shared_ptr<gtsam::NonlinearFactorGraph> new_graph_;

    // State values
    gtsam::Values state_;

    // Tracking variables
    uint64_t last_kf_id_;
    uint64_t curr_kf_id_;

    // Stereo cal
    const gtsam::Cal3_S2Stereo::shared_ptr stereo_cal_;

};

} // namespace vio

} // namespace nightwing

#endif