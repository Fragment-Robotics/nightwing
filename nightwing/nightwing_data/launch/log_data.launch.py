from datetime import datetime
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from simple_launch import SimpleLauncher


sl = SimpleLauncher()

sl.declare_arg('topics_filename', default_value='default_log_topics.yaml', description='YAML file to use for topics to log.')
sl.declare_arg('bag_name', default_value=datetime.now().strftime("%Y-%m-%d-%H-%M-%S"), description="Name of recorded rosbag folder")

def opaque_setup():

  # Config loading
  log_topics_file = os.path.join(
    get_package_share_directory('nightwing_data'),
    'config/' + sl.arg('topics_filename')
  )

  with open(log_topics_file, "r") as f:
    topics = yaml.safe_load(f)["topics"]

  sl.add_action(
    ExecuteProcess(
          cmd=[
            'ros2', 'bag', 'record',
            '-o', '/data/nightwing/rosbags/' + sl.arg('bag_name')
          ] + topics,
          output='screen'
        )
  )

  return sl.launch_description()

generate_launch_description = sl.launch_description(opaque_function=opaque_setup)
  
