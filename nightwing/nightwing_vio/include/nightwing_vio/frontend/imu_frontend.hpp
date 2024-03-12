#ifndef NIGHTWING_VIO_IMU_FRONTEND_HPP
#define NIGHTWING_VIO_IMU_FRONTEND_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <yaml-cpp/yaml.h>

#include <nightwing_vio/common/vio_types.hpp>

namespace nightwing {

namespace vio {

using ImuMeasurement = Eigen::Matrix<double, 6, 1>;
using ImuMeasurements = Eigen::Matrix<double, 6, Eigen::Dynamic>;

// Class for managing the IMU frontend params
//! Assumes that we get time-synchronized IMU measurements with the camera frames.
//! If this changes will need to add time shift parameter.
//! Also doesn't handle combined measurements (from camera) yet.
class ImuFrontendParams {
  public:
    ImuFrontendParams() = default;
    ~ImuFrontendParams() = default;

  public:
    // Parse YAML file for IMU frontend params
    void parseYamlFile(const std::string& file_path);
    // Parse YAML node for IMU frontend params
    void parseYamlParams(const YAML::Node& imu_params);

  private:
    // Initial IMU bias
    double imu_bias_ = gtsam::imuBias::ConstantBias();

    // Noise and random walk parameters
    double accel_noise_sigma_ = 0.0;
    double accel_random_walk_ = 0.0;
    double gyro_noise_sigma_ = 0.0;
    double gyro_random_walk_ = 0.0;
    double integration_sigma_ = 0.0;
    // double imu_time_shift_ = 0.0; 
    
    // Gravity vector in navigation frame
    gtsam::Vector3 n_gravity_ = gtsam::Vector3(0.0, 0.0, 0.0);

}; // class ImuFrontendParams


// Class for managing history of IMU measurements
class ImuMeasurementsBuffer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuMeasurementsBuffer() = default;
    
  public:

    /*
    * @brief Add IMU measurement to buffer
    * @param imu_measurement IMU measurement
    * @return void
    */
    void addMeasurement(const ImuMeasurement& imu_measurement);

    // Get IMU measurements
    ImuMeasurements getMeasurements() const;

  private:

    // IMU measurements
    ImuMeasurements imu_measurements_;

};  // class ImuBuffer


// Class for performing IMU frontend operations.
// Tasks include:
// 1. Measurement tracking in rolling buffer
// 2. IMU pre-integration for factor graph
class ImuFrontend {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PiUPtr = std::unique_ptr<gtsam::PreintegratedImuMeasurements>;
    using CombinedPiUPtr = std::unique_ptr<gtsam::PreintegratedCombinedMeasurements>;

    ImuFrontend(const ImuFrontendParams& imu_params, const gtsam::imuBias::ConstantBias& initial_imu_bias);
    ~ImuFrontend() = default;

  public:

    // Initialize IMU frontend.
    void initialize(const gtsam::imuBias::ConstantBias& imu_bias);

    // Add IMU measurement to buffer. This occurs at Frame rate not 
    // Keyframe rate.
    void addImuMeasurement(
      TimeStamp timestamp,
      const ImuMeasurement& imu_measurement
    );

    // Get IMU bias
    inline gtsam::imuBias::ConstantBias getImuBias() const { return curr_imu_bias_; }

    // Convert frontend params to GTSAM params
    gtsam::PreintegrationMeasurements::Params imuFrontendParamsToGtsamParams(const ImuFrontendParams& params);

    // Update the IMU bias from a recent backend optimization
    inline updateImuBias(const gtsam::imuBias::ConstantBias& new_imu_bias) { curr_imu_bias_ = new_imu_bias; }

    // Reset the pre-integration object with the updated bias. 
    inline void resetIntegrationWithNewBias(const gtsam::imuBias::ConstantBias& new_imu_bias) {
      curr_imu_bias_ = new_imu_bias;
      pi_->resetIntegrationAndSetBias(new_imu_bias);
    }

    // Pre-integrate IMU measurements
    gtsam::PreintegratedImuMeasurements preintegrateImuMeasurements(
      const Eigen::Matrix<double, 1, Eigen::Dynamic>& imu_timestamps,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements,
    );

  private:

    // Parameters for frontend
    ImuFrontendParams params_;

    // IMU measurements
    ImuMeasurementsBuffer imu_buffer_;

    // IMU bias
    gtsam::imuBias::ConstantBias curr_imu_bias_;

    // GTSAM pre-integration class. Currently only supports non-combined measurements.
    // TODO: Add make this a base class type and add dynamic allocation through type casting.
    PiUPtr pi_;

}; // class ImuFrontend

} // namespace vio

} // namespace nightwing

#endif
