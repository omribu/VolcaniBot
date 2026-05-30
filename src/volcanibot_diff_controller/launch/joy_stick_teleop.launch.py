import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("volcanibot_diff_controller")

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="false")
    use_sim_arg = DeclareLaunchArgument(name="use_sim", default_value="false")
    joy_teleop_config_arg = DeclareLaunchArgument(
        "joy_teleop_config",
        default_value=os.path.join(pkg, "config", "joy_teleop.yaml"),
    )
    twist_mux_topics_config_arg = DeclareLaunchArgument(
        "twist_mux_topics_config",
        default_value=os.path.join(pkg, "config", "twist_mux_topics.yaml"),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(pkg, "config", "joy_config.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            LaunchConfiguration("joy_teleop_config"),
            {"use_sim_time": use_sim_time},
        ],
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", "/volcanibot_controller/cmd_vel_unstamped")],
        parameters=[
            {"use_sim_time": use_sim_time},
            os.path.join(pkg, "config", "twist_mux_locks.yaml"),
            LaunchConfiguration("twist_mux_topics_config"),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_arg,
        joy_teleop_config_arg,
        twist_mux_topics_config_arg,
        joy_node,
        joy_teleop,
        twist_mux_node,
    ])
