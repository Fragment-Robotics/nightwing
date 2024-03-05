#ifndef NIGHTWING_VIO_BACKEND_FACTOR_GRAPH_MANAGER_HPP
#define NIGHTWING_VIO_BACKEND_FACTOR_GRAPH_MANAGER_HPP

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"

namespace nightwing {

namespace vio {

struct KeyFrame {
    gtsam::Pose3 pose;
    gtsam::Point3 point;
};

class FactorGraphManager {
    public:
        FactorGraphManager() = default;
        ~FactorGraphManager() = default;

        void addKeyFrameAndOptimize(const KeyFrame& key_frame);

    private:

        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values initial_state_;
        gtsam::Values opt_state_;

}

} // namespace vio

} // namespace nightwing

#endif