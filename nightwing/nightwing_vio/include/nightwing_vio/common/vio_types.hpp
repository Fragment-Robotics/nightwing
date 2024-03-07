#ifndef NIGHTWING_VIO_TYPES_HPP
#define NIGHTWING_VIO_TYPES_HPP

#include <cstdint>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Matrix.h>

#include <opencv2/core.hpp>

namespace nightwing {

namespace vio {

using Timestamp = std::int64_t;

// GTSAM type simplications
using Pose3 = gtsam::Pose3;
using Point2 = gtsam::Point2;
using Point3 = gtsam::Point3;
using Vector3 = gtsam::Vector3;
using Vector6 = gtsam::Vector6;
using Matrix3 = gtsam::Matrix33;
using Matrix6 = gtsam::Matrix66;
using Matrices3f = std::vector<Matrix3, Eigen::aligned_allocator<Matrix3>>;
using Vectors3f = std::vector<Vector3, Eigen::aligned_allocator<Vector3>>;

using KeyFrameId = std::uint64_t;
using LandmarkId = std::uint64_t;

using KeyPoint = cv::Point2f;
using KeyPoints = std::vector<KeyPoint>;


} // namespace vio

} // namespace nightwing

#endif