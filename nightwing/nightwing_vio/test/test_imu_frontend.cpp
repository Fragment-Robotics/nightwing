#include <gtest/gtest.h>

#include <nightwing_vio/frontend/imu_frontend.hpp>


using namespace nightwing::vio;

/*
* ---------------------------------------------------------
* IMU Frontend Params
* ---------------------------------------------------------
*/

TEST(ImuFrontendParamsTest, ParseYamlFile) {
    ImuFrontendParams params;
    params.parseYamlFile("data/imu_params_test.yaml");

    // Check the parsed values
    EXPECT_DOUBLE_EQ(params.accel_noise_sigma_, 0.1);
    EXPECT_DOUBLE_EQ(params.accel_random_walk_, 0.1);
    EXPECT_DOUBLE_EQ(params.gyro_noise_sigma_, 0.1);
    EXPECT_DOUBLE_EQ(params.gyro_random_walk_, 0.1);
    EXPECT_DOUBLE_EQ(params.integration_sigma_, 0.1);
    EXPECT_EQ(params.n_gravity_, gtsam::Vector3(0, 0, 9.81));
    EXPECT_EQ(params.imu_bias_, gtsam::imuBias::ConstantBias(gtsam::Vector6(0, 0, 0, 0, 0, 0)));
}

TEST(ImuFrontendParamsTest, ParseYamlParams) {
    ImuFrontendParams params;
    YAML::Node imu_params;
    imu_params["accel_noise_sigma"] = 0.1;
    imu_params["accel_random_walk"] = 0.1;
    imu_params["gyro_noise_sigma"] = 0.1;
    imu_params["gyro_random_walk"] = 0.1;
    imu_params["integration_sigma"] = 0.1;
    imu_params["n_gravity"] = 9.81;
    imu_params["imu_bias"]["accel_bias"]["x"] = 0;
    imu_params["imu_bias"]["accel_bias"]["y"] = 0;
    imu_params["imu_bias"]["accel_bias"]["z"] = 0;
    imu_params["imu_bias"]["gyro_bias"]["x"] = 0;
    imu_params["imu_bias"]["gyro_bias"]["y"] = 0;
    imu_params["imu_bias"]["gyro_bias"]["z"] = 0;

    params.parseYamlParams(imu_params);

    // Check the parsed values
    EXPECT_DOUBLE_EQ(params.accel_noise_sigma_, 0.1);
    EXPECT_DOUBLE_EQ(params.accel_random_walk_, 0.1);
    EXPECT_DOUBLE_EQ(params.gyro_noise_sigma_, 0.1);
    EXPECT_DOUBLE_EQ(params.gyro_random_walk_, 0.1);
    EXPECT_DOUBLE_EQ(params.integration_sigma_, 0.1);
    EXPECT_EQ(params.n_gravity_, gtsam::Vector3(0, 0, 9.81));
    EXPECT_EQ(params.imu_bias_, gtsam::imuBias::ConstantBias(gtsam::Vector6(0, 0, 0, 0, 0, 0)));
}

/*
* ---------------------------------------------------------
* IMU Measurements Buffer
* ---------------------------------------------------------
*/


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
