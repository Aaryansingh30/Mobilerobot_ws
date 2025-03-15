import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    robot_name = "MobileBot"
    package_name = "mobilerobot_description"

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            get_package_share_directory(package_name),
            'urdf', 'mobilerobot.urdf.xacro'
        ),
        description='Absolute path to robot URDF file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

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
                get_package_share_directory(package_name),
                'config', 'mobilerobot_nav2_stack_rviz.rviz2.rviz'
            )
        ],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'mobilerobot_world.world')}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', '/robot_description',
            # '-x', '0.0',
            # '-y', '0.0',
            # '-z', '0.0',
        ],
        output='screen'
    )

    teleop_keyboard = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    # Run SLAM Toolbox as a raw command
    slam_toolbox_launch = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'slam_params_file:=./src/mobilerobot_nav2_stack/config/mapper_params_online_async.yaml',
            'use_sim_time:=true'
        ],
        output='screen'
    )

    # Run Nav2 Bringup as a raw command
    nav2_bringup_launch = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=true'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gazebo_launch,
        spawn_entity,
        teleop_keyboard,
        slam_toolbox_launch, 
        nav2_bringup_launch 
    ])
