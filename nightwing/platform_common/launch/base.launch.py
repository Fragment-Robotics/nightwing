from launch.actions import ExecuteProcess
from simple_launch import SimpleLauncher


def generate_launch_description():

  sl = SimpleLauncher()

  # Launch arguments
  sl.declare_arg('compress_images', default_value=False, description="Enable image compression via isaac ros node")

  # Load camera launch
  sl.include('nightwing_platform_common', 'camera.launch.py', launch_arguments={'compress_images': sl.arg('compress_images')})

  # Opens the PX4-to-Jetson ROS2 bridge
  sl.add_action(
    ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
      )
  )

  return sl.launch_description()