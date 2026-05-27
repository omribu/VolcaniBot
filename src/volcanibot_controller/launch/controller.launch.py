import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    agribot_description_dir = get_package_share_directory("agribot_description")


    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(agribot_description_dir, "urdf", "agribot_ign.urdf"),
        description="Absolute path to robot URDF file"
    )    

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}]
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
    )


    wheel_controller_spawner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "volcanibot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )






    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        robot_state_publisher,
        # rviz_node,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
    ])