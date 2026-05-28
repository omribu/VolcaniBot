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
        get_package_share_directory("agribot_description"),
        "urdf",
        "zed_robot_agribot.xacro"
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

    # Arguments
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
    initial_camera_arg = DeclareLaunchArgument(
        "initial_camera",
        default_value="right_camera",
        description="Initial camera to use (right_camera or left_camera)"
    )

    zed_override_real_path = os.path.join(
    get_package_share_directory("agribot_bringup"),
    "config",
    "zed_override_real.yaml"
    )

    # Gains for Line following controller
    kp_heading_arg = DeclareLaunchArgument("kp_heading", default_value="0.7")
    kd_heading_arg = DeclareLaunchArgument("kd_heading", default_value="0.8")
    ki_heading_arg = DeclareLaunchArgument("ki_heading", default_value="0.0")



    use_sim_time   = LaunchConfiguration("use_sim_time")
    use_sim        = LaunchConfiguration("use_sim")
    initial_camera = LaunchConfiguration("initial_camera")

    agribot_controller_pkg = get_package_share_directory('agribot_controller')

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("agribot_diff_controller"),
            "launch",
            "diffdrive.launch.py"
        )
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("agribot_controller"),
            "launch",
            "joy_stick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim":      use_sim,
            "use_sim_time": use_sim_time,
            # "joy_teleop_config": os.path.join(
            #     agribot_controller_pkg, "config", "joy_teleop_real.yaml"
            # ),
            # "twist_mux_topics_config": os.path.join(
            #     agribot_controller_pkg, "config", "twist_mux_topics_real.yaml"
            # ),
        }.items()
    )


    nav_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("agribot_navigation"),
            "launch",
            "real_nav.launch.py"
        ),
        launch_arguments={
            "initial_camera": LaunchConfiguration("initial_camera"),
            "use_sim_time": "False"
        }.items()
    )


    # ---------------- ZED ----------------
    zed_wrapper_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py"
            )
        ),
        launch_arguments={
            "camera_model": "zedx",
            "publish_tf": "false",
            "publish_urdf": "false",  # our RSP (OpaqueFunction) handles robot_description
            "ros_params_override_path": zed_override_real_path,
        }.items()
    )


    # ---------------- RealSense (right) ----------------
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'camera_name':  'realsense_right',   # topic: /camera/realsense_right/color/image_raw
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_sync': 'false',
            'publish_tf': 'true', 
        }.items()
    )

    # Bridges URDF frame (camera_right_link) to the RealSense driver's root frame (realsense_right_link).
    # The driver owns realsense_right_link and its children; the URDF owns camera_right_link.
    # Setting base_frame_id on the driver would cause a broadcaster conflict, so we use a static TF instead.
    realsense_right_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_right_tf_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_right_link', 'realsense_right_link'],
        output='screen',
    )






    # Joint state publisher — publishes zero values for wheel joints when controller is off.
    # Comment out when uncommenting `controller` above (the joint_state_broadcaster takes over).
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("agribot_description"), "rviz", "real_robot.rviz")]
    )


    return LaunchDescription([
        use_sim_time_arg,
        use_sim_arg,
        initial_camera_arg,
        OpaqueFunction(function=launch_robot_state_publisher),
        joint_state_pub,
        # controller,  # uncomment when Roboteq motor driver is connected at /dev/ttyACM0
        joystick,
        zed_wrapper_node,
        realsense,
        realsense_right_tf_bridge,
        rviz_node,
    ])