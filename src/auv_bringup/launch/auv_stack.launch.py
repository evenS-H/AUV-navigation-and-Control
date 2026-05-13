from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    thruster_vectoring = Node(
        package="Thruster_Vectoring",
        executable="Thruster_Vectoring",
        name="thruster_vectoring",
        output="screen",
    )

    return LaunchDescription([
        # Actuator interface needs MAVROS — small delay so MAVROS/services are ready
        TimerAction(period=2.0, actions=[thruster_vectoring]),

        Node(
            package="LOS",
            executable="LOS",
            name="los",
            output="screen",
        ),

        Node(
            package="PID",
            executable="PID",
            name="pid",
            output="screen",
        ),

        Node(
            package="Mission_Manager",
            executable="Mission_Manager",
            name="mission_manager",
            output="screen",
        ),

        Node(
            package="World_Click_Map",
            executable="World_Click_Map",
            name="world_click_map",
            output="screen",
            parameters=[{"port": 8010}],
        ),
    ])
