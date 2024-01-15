import os
import yaml

from ament_index_python.packages import get_package_share_directory
from simple_launch import SimpleLauncher


def generate_launch_description():
    """Launch description for spinning up realsense camera."""

    sl = SimpleLauncher()

    # Launch arguments
    sl.declare_arg('compress_images', default_value=False, description="Enable image compression via isaac ros node")

    # Configs
    realsense_config_file_path = os.path.join(
        get_package_share_directory('nightwing_platform_common'),
        'config/perception/realsense.yaml'
    )
    with open(realsense_config_file_path, "r") as f:
        realsense_config = yaml.safe_load(f)
        (width, height, fps) = [int(x) for x in realsense_config['rgb_camera']['profile'].split('x')]

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
    with sl.group(ns='nw_perception', if_arg='compress_images'):
        with sl.container(name='/camera_container', existing=True):
            sl.node(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='depth_encoder_node',
                parameters=[{
                    'input_width': width,
                    'input_height': height,
                }],
                remappings=[
                    ('image_raw', '/camera/realsense_camera/depth/image_rect_raw'),
                    ('image_compressed', '/nw/perception/depth/image_rect_compressed'),
                ],
            )
            sl.node(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='color_encoder_node',
                parameters=[{
                    'input_width': width,
                    'input_height': height,
                }],
                remappings=[
                    ('image_raw', '/camera/realsense_camera/color/image_raw'),
                    ('image_compressed', '/nw/perception/color/image_compressed'),
                ],
            )

    return sl.launch_description()