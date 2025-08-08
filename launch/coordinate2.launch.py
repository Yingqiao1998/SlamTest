import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ws_dir = get_package_share_directory('coordinate').split('install')[0]
coordinate_drone1_cfg = os.path.join(ws_dir, 'src', 'coordinate', 'cfg',
    'coordinate_drone2.yaml')
print("load coordinate_drone1_cfg: {}".format(coordinate_drone1_cfg))

def generate_launch_description():
    container = ComposableNodeContainer(
        name="coordinate_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="coordinate",
                plugin="coordinate_composition::CoordinateComponent",
                name="coordinate_component",
                namespace='',
                parameters=[coordinate_drone1_cfg],
            )
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])
