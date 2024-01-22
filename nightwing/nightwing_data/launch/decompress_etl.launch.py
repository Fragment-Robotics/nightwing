import launch

from simple_launch import SimpleLauncher
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Launch the H.264 Decoder Node against ros bag."""

    sl = SimpleLauncher()

    sl.declare_arg('extract_to', default_value="/data/extracted_images", description="Directory to extract uncompressed images too.")
    sl.declare_arg('time_diff_s', default_value=2, description="Logging frequency in seconds.")

    with sl.container(name="decompression_container"):
        sl.node(
            name="color_node",
            package="isaac_ros_h264_decoder",
            plugin="nvidia::isaac_ros::h264_decoder::DecoderNode",
            remappings=[
                ('image_compressed', '/nw/perception/color/image_compressed'),
                ('image_uncompressed', '/color/image_uncompressed')
            ]
        )
        sl.node(
            name="depth_node",
            package="isaac_ros_h264_decoder",
            plugin="nvidia::isaac_ros::h264_decoder::DecoderNode",
            remappings=[
                ('image_compressed', '/nw/perception/depth/image_rect_compressed'),
                ('image_uncompressed', '/depth/image_uncompressed')
            ]
        )
        sl.node(
            name="sync_store_node",
            package="nightwing_data",
            plugin="nightwing::data::RGBDSyncStore",
            parameters=[{
                'directory_path': sl.arg('extract_to'),
                'time_diff_s': sl.arg('time_diff_s')
            }]
        )

    return sl.launch_description()