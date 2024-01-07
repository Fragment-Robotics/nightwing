import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description with multiple components"""
  container = ComposableNodeContainer(
    name="example_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
      ComposableNode(
        package="example_package",
        plugin="nightwing::Talker",
        name="example_talker"
      ),
      ComposableNode(
        package="example_package",
        plugin="nightwing::Listener",
        name="example_listener"
      )
    ],
    output="both"
  )

  return launch.LaunchDescription([container])