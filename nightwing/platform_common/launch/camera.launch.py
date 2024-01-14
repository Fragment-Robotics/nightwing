import os

from ament_index_python.packages import get_package_share_directory
from simple_launch import SimpleLauncher


def generate_launch_description():
    """Launch description for spinning up realsense camera."""

    sl = SimpleLauncher()

    # Launch arguments
    log_data = sl.declare_arg('log_data', default_value=False, description="Log data to rosbag")

    # Configs
    realsense_config_file_path = os.path.join(
        get_package_share_directory('nightwing_platform_common'),
        'config/perception/realsense.yaml'
    )

    # Create camera container and load realsense node
    with sl.container(name='camera_container'):
        sl.node(
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            name='realsense_camera',
            parameters=[realsense_config_file_path],
            extra_arguments=[
                {'use_intra_process_comms': True}
            ]
        )

    # Logging data block. spin up h264 encoder blocks for both depth and color.
    # Note that for this to work you need to have the colorization filter enabled
    # for the realsense so we get values in rgb format. 
    with sl.group(if_arg='log_data'):
        with sl.container(name='/camera_container', existing=True):
            sl.node(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='depth_encoder_node',
                parameters=[{
                    'input_width': 1280,
                    'input_height': 720,
                }],
                remappings=[
                    ('image_raw', '/depth/image_rect_raw'),
                    ('image_compressed', '/depth/image_rect_compressed'),
                ],
            )
            sl.node(
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

    return sl.launch_description()