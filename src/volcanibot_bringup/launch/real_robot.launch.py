import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_robot_state_publisher(context, *args, **kwargs):
    xacro_file = os.path.join(
        get_package_share_directory("volcanibot_description"),
        "urdf",
        "volcanibot.xacro"
    )
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={"use_sim": "false"}
    ).toxml()

    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_xml,
            "use_sim_time": False,
        }]
    )]


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Use simulation"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim = LaunchConfiguration("use_sim")

    # Joystick teleop (joy_node + joy_teleop + twist_mux) 
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("volcanibot_diff_controller"),
            "launch",
            "joy_stick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim": use_sim,
            "use_sim_time": use_sim_time,
        }.items()
    )

    # RealSense D435 (rear) 
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'camera_name': 'realsense_rear',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_sync': 'false',
            'publish_tf': 'true',
            'align_depth.enable': 'true',
        }.items()
    )

    realsense_rear_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_rear_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'rear_camera_link', 'realsense_rear_link'],
        output='screen',
    )

    # YOLO detection + tracking + 3D 
    yolo_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("yolo_bringup"),
            "launch",
            "yolo.launch.py"
        ),
        launch_arguments={
            'use_3d': 'True',
            'use_tracking': 'True',
            'model': '/home/volcani/workspaces/VolcaniBot/models/yolo11m.engine',
            'tracker': 'botsort.yaml',
            'device': 'cuda:0',
            'image_reliability': '2',
            'half': 'True',
            'imgsz_height': '480',
            'imgsz_width': '640',
            'input_image_topic': '/camera/realsense_rear/color/image_raw',
            'input_depth_topic': '/camera/realsense_rear/aligned_depth_to_color/image_raw',
            'input_depth_info_topic': '/camera/realsense_rear/aligned_depth_to_color/camera_info',
            'depth_image_units_divisor': '1000',
            'target_frame': 'base_link',
        }.items()
    )

    # Follow-person node (button toggle + closest-person tracking)
    follow_person_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("volcanibot_follow_person"),
            "launch",
            "follow_person.launch.py"
        )
    )

    # Joint state publisher — publishes zero values for wheel joints when
    # the hardware controller (diffdrive) is not running.
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("volcanibot_diff_controller"),
            "launch",
            "diffdrive.launch.py"
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("volcanibot_description"), "rviz", "real_robot.rviz"
        )]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_sim_arg,
        OpaqueFunction(function=launch_robot_state_publisher),
        # joint_state_pub,
        controller,  # uncomment when Roboteq motor driver is connected at /dev/ttyACM0
        joystick,
        realsense,
        realsense_rear_tf_bridge,
        yolo_launch,
        follow_person_launch,
        rviz_node,
    ])
