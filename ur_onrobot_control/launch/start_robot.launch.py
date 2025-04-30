from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    robot_ip = LaunchConfiguration("robot_ip")

    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('ur_onrobot_description'), "urdf", 'ur_onrobot.urdf.xacro']),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "name:=",
            "ur_onrobot",
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare('ur_onrobot_control'), "config", 'ur_onrobot_controllers.yaml']
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur_onrobot_description'), "rviz", "view_robot.rviz"]
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "config",
            ur_type.perform(context) + "_update_rate.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(
            AndSubstitution(launch_dashboard_client, NotSubstitution(use_fake_hardware))
        ),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        parameters=[
            {"headless_mode": headless_mode},
        ],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": 54321,
                "device_name": "/tmp/ttyUR",
            }
        ],
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
            {"joint_controller_active": activate_joint_controller},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawn controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
        "finger_width_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
    ]
    if activate_joint_controller.perform(context) == "true":
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))

    if use_fake_hardware.perform(context) == "true":
        controllers_active.remove("tcp_pose_broadcaster")

    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    nodes_to_start = [
        control_node,
        ur_control_node,
        dashboard_client_node,
        robot_state_helper_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        robot_state_publisher_node,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur3e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "onrobot_type",
            description="Type/series of used OnRobot gripper.",
            choices=["rg2", "rg6"],
            default_value="rg2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
            default_value="192.168.56.101",  # Uses the Polyscope sim by default
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
