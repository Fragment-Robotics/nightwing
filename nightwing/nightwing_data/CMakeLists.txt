cmake_minimum_required(VERSION 3.8)
project(nightwing_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(image_transport REQUIRED)
find_package(zed_components)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(rgbd_sync_store SHARED
  src/rgbd_sync_store.cpp
)
ament_target_dependencies(rgbd_sync_store
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  message_filters
)
rclcpp_components_register_nodes(rgbd_sync_store "nightwing::data::RGBDSyncStore")

add_library(zed_rgb_converter SHARED
  src/zed_rgb_converter.cpp
)
ament_target_dependencies(zed_rgb_converter
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  zed_components
  image_transport
)
rclcpp_components_register_nodes(zed_rgb_converter "nightwing::data::ZEDRGBConverter")

install(TARGETS
  rgbd_sync_store
  zed_rgb_converter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/decompress_etl_entrypoint.sh
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
