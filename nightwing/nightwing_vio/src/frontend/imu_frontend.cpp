#include "nightwing/nightwing_vio/frontend/imu_frontend.hpp"
#include "nightwing/nightwing_vio/common/vio_types.hpp"

namespace nightwing {

namespace vio {

/*
* ---------------------------------------------------------
* IMU Frontend Params
* ---------------------------------------------------------
*/

void ImuFrontendParams::parseYamlFile(const std::string& file_path) {
    // Parse the YAML file
    YAML::Node imu_params = YAML::LoadFile(file_path);
    parseYamlParams(imu_params["imu_params"]);
}

void ImuFrontendParams::parseYamlParams(const YAML::Node& imu_params) {
    // Parse the IMU frontend params
    accel_noise_sigma_ = imu_params["accel_noise_sigma"].as<double>();
    accel_random_walk_ = imu_params["accel_random_walk"].as<double>();
    gyro_noise_sigma_ = imu_params["gyro_noise_sigma"].as<double>();
    gyro_random_walk_ = imu_params["gyro_random_walk"].as<double>();
    integration_sigma_ = imu_params["integration_sigma"].as<double>();
    n_gravity_ = gtsam::Vector3(0, 0, imu_params["n_gravity"].as<double>());

    // Convert to GTSAM imuBias::ConstantBias
    Vector6 imu_bias_vec(
        imu_params["imu_bias"]["accel_bias"]["x"].as<double>(),
        imu_params["imu_bias"]["accel_bias"]["y"].as<double>(),
        imu_params["imu_bias"]["accel_bias"]["z"].as<double>(),
        imu_params["imu_bias"]["gyro_bias"]["x"].as<double>(),
        imu_params["imu_bias"]["gyro_bias"]["y"].as<double>(),
        imu_params["imu_bias"]["gyro_bias"]["z"].as<double>()
    );
    imu_bias_ = gtsam::imuBias::ConstantBias(imu_bias_vec);
}

/*
* ---------------------------------------------------------
* IMU Measurements Buffer
* ---------------------------------------------------------
*/

void ImuMeasurementsBuffer::addMeasurement(const ImuMeasurement& imu_measurement) {
    // TODO
}



/*
* ---------------------------------------------------------
* IMU Frontend
* ---------------------------------------------------------
*/

ImuFrontend::ImuFrontend(const ImuFrontendParams& params) : params_(params) {
    // TODO: Add checks on our params
    // Initialize the frontend
    initialize(params.imu_bias_);
}

gtsam::PreintegrationMeasurements::Params ImuFrontend::imuFrontendParamsToGtsamParams(const ImuFrontendParams& params) {
    gtsam::PreintegrationMeasurements::Params pi_params;
    pi_params.accelerometerCovariance = std::pow(params.accel_noise_sigma_, 2) * gtsam::I_3x3;
    pi_params.gyroscopeCovariance = std::pow(params.gyro_noise_sigma_, 2) * gtsam::I_3x3;
    pi_params.integrationCovariance = std::pow(params.integration_sigma_, 2) * gtsam::I_3x3;
    pi_params.n_gravity = params.n_gravity_;
    pi_params.use2ndOrderCoriolis = false;

    return pi_params;
}

void ImuFrontend::initialize(const gtsam::imuBias::ConstantBias& imu_bias) {
    // Covert our frontend params to GTSAM params for non-combined measurements
    auto pi_params = imuFrontendParamsToGtsamParams(params_);

    // Create preintegration measurements object. Once we have optional combined measurements, we will need to update this
    // to call the appropriate setup function.
    pi_ = std::make_unique<gtsam::PreintegrationMeasurements>(pi_params, imu_bias);

    // Update our current bias
    curr_imu_bias_ = imu_bias;
}

void ImuFrontend::preintegrateImuMeasurements(
    const Eigen::Matrix<double, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements
) {
    // Check if we have any measurements
    if (imu_timestamps.size() == 0) {
        return;
    }

    // Check if we have the correct number of measurements
    if (imu_timestamps.cols() != imu_measurements.cols()) {
        throw std::runtime_error("IMU timestamps and measurements are not the same size.");
    }

    // Split our measurements into accelerometer and gyroscope
    Eigen::Matrix<double, 3, Eigen::Dynamic> accel_measurements = imu_measurements.topRows(3);
    Eigen::Matrix<double, 3, Eigen::Dynamic> gyro_measurements = imu_measurements.bottomRows(3);

    // Convert timestamps to deltas in seconds
    Eigen::Matrix<double, 1, Eigen::Dynamic> delta_ts = imu_timestamps.rightCols(imu_timestamps.cols() - 1) - imu_timestamps.leftCols(imu_timestamps.cols() - 1);

    // Preintegrate our measurements
    pi_->integrateMeasurements(accel_measurements, gyro_measurements, delta_ts);
}

} // namespace nightwing::vio

} // namespace nightwing::vio