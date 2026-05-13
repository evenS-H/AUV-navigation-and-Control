from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mavros_params = PathJoinSubstitution(
        [FindPackageShare("auv_bringup"), "config", "mavros_apm_filtered.yaml"]
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        parameters=[
            {"fcu_url": "tcp://127.0.0.1:5777/?ids=255,240"},
            {"fcu_rate": 10.0},
            mavros_params,
        ],
    )

    pressure_node = Node(
        package="Pressure",
        executable="Pressure",
        name="pressure",
        output="screen",
        parameters=[{
            "connection_url": "tcp:127.0.0.1:5777",
            "pressure_topic": "/bar30/pressure",
            "frame_id": "bar30_link",
        }],
    )

    dvl_node = Node(
        package="DVL",
        executable="DVL",
        name="dvl",
        output="screen",
    )

    navigation_node = Node(
        package="Navigation",
        executable="Navigation",
        name="navigation",
        output="screen",
    )

    return LaunchDescription([
        mavros_node,
        TimerAction(period=6.0,  actions=[pressure_node]),
        TimerAction(period=8.0,  actions=[dvl_node]),
        TimerAction(period=10.0, actions=[navigation_node]),
    ])
