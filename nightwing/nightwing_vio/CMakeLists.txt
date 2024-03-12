cmake_minimum_required(VERSION 3.8)
project(nightwing_vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_frontend test/test_imu_frontend.cpp)
  target_include_directories(test_imu_frontend PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_imu_frontend
    std_msgs
  )
  target_link_libraries(test_imu_frontend name_of_local_library)
endif()

ament_package()
