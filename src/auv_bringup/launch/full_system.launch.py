from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('auv_bringup'), 'launch', 'sensor_only.launch.py'])
        )
    )

    auv_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('auv_bringup'), 'launch', 'auv_stack.launch.py'])
        )
    )

    route_plot = Node(
        package='route_plot',
        executable='mission_odom_logger',
        name='mission_odom_logger',
        output='screen',
        parameters=[
            {'odom_topic': '/odometry/filtered'},
            {'mission_topic': '/auv_route_enu'},
        ],
    )

    return LaunchDescription([
       # route_plot,
        TimerAction(period=2.0, actions=[sensors]),
        TimerAction(period=14.0, actions=[auv_stack]),
    ])