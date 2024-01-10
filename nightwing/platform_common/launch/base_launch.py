import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  # NOTE: When the size of the project becomes larger the right way to organize
  # would be to have launch files defined for each module of the stack i.e.,
  # localization, panoptic seg, mapping, etc and the platform_common launches
  # would load those.
  rosbag_path = LaunchConfiguration('rosbag_path', default=f'/data/rosbags/{datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}')

  realsense_config_file_path = os.path.join(
    get_package_share_directory('nightwing_platform_common'),
    'config/vision/realsense.yaml'
  )

  # If we want to log data, we also spin up the image processing nodes
  # to save the data in a compressed format (H.264). The IfCondition
  # is used to only spin up the nodes if the log_data argument
  # is set to true.
  vision_container = ComposableNodeContainer(
    name='vision_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='realsense2_camera',
        namespace='',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[realsense_config_file_path],
        extra_arguments=[
          {'use_intra_process_comms': True}
        ]
      ),
      ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='depth_format_node',
        parameters=[{
          'encoding_desired': 'rgb8',
        }],
        remappings=[
          ('image_raw', '/depth/image_rect_raw'),
          ('image', '/depth/image_rect_rgb8'),
        ],
      ),
      ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageEncoderNode',
        name='depth_encoder_node',
        parameters=[{
          'input_width': 1280,
          'input_height': 720,
        }],
        remappings=[
          ('image_raw', '/depth/image_rect_rgb8'),
          ('image_compressed', '/depth/image_rect_compressed'),
        ],
      ),
      ComposableNode(
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        name='color_encoder_node',
        parameters=[{
          'input_width': 1280,
          'input_height': 720,
        }],
        remappings=[
          ('image_raw', '/color/image_raw'),
          ('image_compressed', '/color/image_compressed'),
        ],
      )
    ],
    output='screen',
  )

  # Opens the PX4-to-Jetson ROS2 bridge
  xrce_dds_node = ExecuteProcess(
    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    output='screen'
  )

  # Rosbag recording
  rosbag_record_node = ExecuteProcess(
    cmd=[
      'ros2', 'bag', 'record',
      '-o', rosbag_path,
      '/depth/image_rect_compressed',
      '/color/image_compressed',
    ],
  )

  return LaunchDescription([
    vision_container,
    xrce_dds_node,
    rosbag_record_node
  ])