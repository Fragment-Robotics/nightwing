import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  # NOTE: When the size of the project becomes larger the right way to organize
  # would be to have launch files defined for each module of the stack i.e.,
  # localization, panoptic seg, mapping, etc and the platform_common launches
  # would load those.
  realsense_config_file_path = os.path.join(
    get_package_share_directory('nightwing_platform_common'),
    'config/vision/realsense.yaml'
  )

  realsense_node = ComposableNode(
    package='realsense2_camera',
    namespace='',
    plugin='realsense2_camera::RealSenseNodeFactory',
    parameters=[realsense_config_file_path],
    extra_arguments=[
      {'use_intra_process_comms': True}
    ]
  )

  vision_container = ComposableNodeContainer(
    name='vision_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      realsense_node,
    ],
    output='screen',
  )

  # Opens the PX4-to-Jetson ROS2 bridge
  xrce_dds_node = ExecuteProcess(
    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    output='screen'
  )

  return LaunchDescription([
    vision_container,
    xrce_dds_node
  ])