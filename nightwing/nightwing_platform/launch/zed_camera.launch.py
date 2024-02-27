import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from simple_launch import SimpleLauncher


def generate_launch_description():
    """Launch description for spinning up Zed camera and optional components."""

    sl = SimpleLauncher()

    # Launch arguments
    sl.declare_arg(
        "compress_images",
        default_value=False,
        description="Enable image compression via isaac ros node",
    )

    # ZED Configurations to be loaded by ZED Node. We track our own
    # config files for the ZED camera.
    config_common = os.path.join(
        get_package_share_directory("nightwing_platform"),
        "config/perception",
        "zed_common.yaml",
    )
    config_camera = os.path.join(
        get_package_share_directory("nightwing_platform"),
        "config/perception",
        "zed2.yaml",
    )
    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(get_package_share_directory("zed_wrapper"), "urdf", "zed_descr.urdf.xacro")

    sl.node(
        package="robot_state_publisher",
        namespace="zed",
        executable="robot_state_publisher",
        name="zed_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro",
                        " ",
                        xacro_path,
                        " ",
                        "camera_name:=zed2",
                        " ",
                        "camera_model:=zed2",
                    ]
                )
            }
        ],
    )

    # Create camera container and load ZedCamera component
    with sl.container(name="camera_container"):
        # Zed component
        sl.node(
            package="zed_components",
            namespace="zed",
            plugin="stereolabs::ZedCamera",
            name="front_camera",
            parameters=[
                config_common,
                config_camera,
                {
                    "camera_name": "front_camera",
                },
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    # Additional h264 encoder nodes for color image compression
    # when storing images to disk or streaming over the network.
    # TODO: Determine way to automatically determine image width and height from ZED camera configuration.
    image_width = 1920
    image_height = 1080
    with sl.group(if_arg="compress_images"):
        with sl.container(name="/camera_container", existing=True):
            #
            # Optional: Setup for depth image compression. Would need to implement a
            # custom filter to go from 16 bit depth to 24 bit color so we can use
            # the h264 encoder. Check out https://dev.intelrealsense.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras.
            #
            sl.node(
                package="isaac_ros_h264_encoder",
                plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
                name="left_color_encoder_node",
                parameters=[
                    {
                        "input_width": image_width,
                        "input_height": image_height,
                    }
                ],
                remappings=[
                    ("image_raw", "/zed/front_camera/left/image_rect_color"),
                    (
                        "image_compressed",
                        "/nw/perception/front_camera/left/image_compressed",
                    ),
                ],
            )
            sl.node(
                package="isaac_ros_h264_encoder",
                plugin="nvidia::isaac_ros::h264_encoder::EncoderNode",
                name="right_color_encoder_node",
                parameters=[
                    {
                        "input_width": image_width,
                        "input_height": image_height,
                    }
                ],
                remappings=[
                    ("image_raw", "/zed/front_camera/right/image_rect_color"),
                    (
                        "image_compressed",
                        "/nw/perception/front_camera/right/image_compressed",
                    ),
                ],
            )

    return sl.launch_description()
