import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    volcanibot_controller_pkg = get_package_share_directory('volcanibot_controller')

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True"
    )
    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="True"
    )
    joy_teleop_config_arg = DeclareLaunchArgument(
        "joy_teleop_config",
        default_value=os.path.join(volcanibot_controller_pkg, "config", "joy_teleop.yaml")
    )
    twist_mux_topics_config_arg = DeclareLaunchArgument(
        "twist_mux_topics_config",
        default_value=os.path.join(volcanibot_controller_pkg, "config", "twist_mux_topics.yaml")
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(volcanibot_controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            LaunchConfiguration("joy_teleop_config"),
            {"use_sim_time": use_sim_time}
        ],
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings=[("/cmd_vel_out", "/volcanibot_controller/cmd_vel_unstamped")],
        parameters=[
            {"use_sim_time": use_sim_time},
            os.path.join(volcanibot_controller_pkg, "config", "twist_mux_locks.yaml"),
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






























# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():

#     volcanibot_controller_pkg = get_package_share_directory('volcanibot_controller')

#     use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
#                                       description="Use simulated time"
#     )

#     joy_node = Node(
#         package="joy",
#         executable="joy_node",
#         name="joystick",
#         parameters=[os.path.join(get_package_share_directory("volcanibot_controller"), "config", "joy_config.yaml"),
#                     {"use_sim_time": LaunchConfiguration("use_sim_time")}],
#     )

#     joy_teleop = Node(
#         package="joy_teleop",
#         executable="joy_teleop",
#         parameters=[os.path.join(get_package_share_directory("volcanibot_controller"), "config", "joy_teleop.yaml"),
#                     {"use_sim_time": LaunchConfiguration("use_sim_time")}],
#     )

#     # virtual_joy_teleop = Node(
#     #     package="joy_teleop",
#     #     executable="joy_teleop",
#     #     name="virtual_joy_teleop",          # different name to avoid conflict
#     #     remappings=[("joy", "input/joy")],  # remap input from virtual joystick topic
#     #     parameters=[
#     #         os.path.join(volcanibot_controller_pkg, "config", "virtual_joy_teleop.yaml"),
#     #         {"use_sim_time": LaunchConfiguration("use_sim_time")},
#     #     ],
#     # )

#     twist_mux_launch = IncludeLaunchDescription(
#         os.path.join(
#             get_package_share_directory("twist_mux"),
#             "launch",
#             "twist_mux_launch.py"
#         ),
#         launch_arguments={
#             "cmd_vel_out" : "volcanibot_controller/cmd_vel_unstamped",
#             "config_topics" : os.path.join(volcanibot_controller_pkg, "config", "twist_mux_topics.yaml"),
#             "config_locks" : os.path.join(volcanibot_controller_pkg, "config", "twist_mux_locks.yaml"),
#             "config_joy" : os.path.join(volcanibot_controller_pkg, "config", "twist_mux_joy.yaml"),
#             "use_sim_time": LaunchConfiguration("use_sim_time"),
#         }.items(),
#     )

#     twist_relay_node = Node(
#         package="volcanibot_controller",
#         executable="twist_relay",
#         name="twist_relay",
#         parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
#     )


#     return LaunchDescription([
#         use_sim_time_arg,
#         joy_teleop,
#         joy_node,
#         # virtual_joy_teleop,
#         twist_mux_launch,
#         twist_relay_node
#     ])