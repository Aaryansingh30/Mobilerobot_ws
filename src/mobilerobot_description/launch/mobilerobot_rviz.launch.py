import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    bot_description_dir = get_package_share_directory('mobilerobot_description')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(
            bot_description_dir, 'urdf', 'mobilerobot.urdf.xacro'
        ),
        description='Absolute path to robot URDF file'
    )

    # Define robot description parameter
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),  # Fixed space issue
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
                bot_description_dir,
                'config',
                'mobilerobot_rviz.rviz2.rviz'  
            )
        ],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
