import os
from datetime import datetime
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from simple_launch import SimpleLauncher


def generate_launch_description():

  sl = SimpleLauncher()

  # Launch arguments
  bag_name = sl.declare_arg('bag_name', default_value=datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
  log_data = sl.declare_arg('log_data', default_value=False, description='Log data to rosbag')

  # Config loading
  log_topics_file = os.path.join(
    get_package_share_directory('nightwing_platform_common'),
    'config/log_topics.yaml'
  )

  # Load camera launch
  sl.include('nightwing_platform_common', 'camera.launch.py', launch_arguments={'log_data': sl.arg('log_data')})

  # Opens the PX4-to-Jetson ROS2 bridge
  sl.add_action(
    ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
      )
  )

  with sl.group(if_arg='log_data'):
    # Rosbag recording
    with open(log_topics_file, "r") as f:
      topics = yaml.safe_load(f)["topics"]

    sl.add_action(
      ExecuteProcess(
            cmd=[
              'ros2', 'bag', 'record',
              '-o', '/data/nightwing/rosbags/' + sl.arg('bag_name')
            ] + topics,
          )
    )

  return sl.launch_description()