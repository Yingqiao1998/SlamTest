import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PythonExpression

ws_dir = get_package_share_directory("slam").split("install")[0]
# slam_drone1_cfg = os.path.join(ws_dir, "src", "slam", "cfg", "slam_drone1.yaml")
slam_drone1_cfg = os.path.join(ws_dir, "cfg", "slam_drone1.yaml")

print("load slam_drone1_cfg: {}".format(slam_drone1_cfg))


def generate_launch_description():
    # 可选：none | gdb | gdbserver
    debug_mode_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="none",
        description="Debug mode: none | gdb | gdbserver",
    )
    gdb_port_arg = DeclareLaunchArgument(
        "gdb_port",
        default_value="1234",
        description="Port for gdbserver (when debug_mode=gdbserver)",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="warn", description="ROS 2 log level"
    )

    # 公共节点描述
    node_desc = [
        ComposableNode(
            package="slam",
            plugin="slam_composition::SlamComponent",
            name="slam_component",
            namespace="",
            parameters=[slam_drone1_cfg],
        )
    ]

    # 普通启动
    container_normal = ComposableNodeContainer(
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("debug_mode"), '" == "none"'])
        ),
        name="slam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",  # 单线程容器；需要并发可改 component_container_mt
        composable_node_descriptions=node_desc,
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # 在 gdb 下运行（前台交互式，终端会进入 gdb）
    container_gdb = ComposableNodeContainer(
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("debug_mode"), '" == "gdb"'])
        ),
        name="slam_container_gdb",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=node_desc,
        output="screen",
        prefix=[
            "gdb",
            "--nx",
            "-q",
            "-ex",
            "handle SIG32 pass noprint nostop",  # 避免TLS相关信号干扰
            "--args",
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        emulate_tty=True,
    )

    # 用 gdbserver（推荐在服务器/容器环境，attach 很快）
    container_gdbserver = ComposableNodeContainer(
        condition=IfCondition(
            PythonExpression(
                ['"', LaunchConfiguration("debug_mode"), '" == "gdbserver"']
            )
        ),
        name="slam_container_gdbserver",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=node_desc,
        output="screen",
        prefix="gdbserver localhost:20045 --",  # 先用固定端口验证
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return launch.LaunchDescription(
        [
            debug_mode_arg,
            gdb_port_arg,
            log_level_arg,
            container_normal,
            container_gdb,
            container_gdbserver,
        ]
    )
