import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    robot_name = "MobileBot"
    package_name = "mobilerobot_description"
    slam_package = "ekf_slam"

    model_arg = DeclareLaunchArgument(
        name='model',
    default_value=os.path.join(
        get_package_share_directory('mobilerobot_description'),
        'urdf', 'mobilerobot.urdf.xacro'
    ),
    description='Absolute path to robot URDF file'
    )

    # Define robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Define the nodes to be launched
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('mobilerobot_description'),  # ✅ Corrected
                'config',
                'mobilerobot_ekf_slam_rviz.rviz2.rviz'
            )
        ],
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'mobilerobot_world.world')}.items()
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', '/robot_description',
            '-x', '-6.607740',
            '-y', '0.216009',
            '-z', '0.500000',
        ],
        output='screen'
    )

    # Run teleop_keyboard in a new terminal
    teleop_keyboard = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    # Launch EKF SLAM Node
    ekf_slam_node = Node(
        package=slam_package,
        executable='ekf_slam',
        name='ekf_slam',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gazebo_launch,
        spawn_entity,
        teleop_keyboard,
        ekf_slam_node,  # Added EKF SLAM Node
    ])
