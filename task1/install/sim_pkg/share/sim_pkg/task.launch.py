from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    talker_node = Node(
        package="sim_pkg",
        executable="simulator",
    )
    listener_node = Node(
        package="sim_pkg",
        executable="sender"
    )
    listener_node2 = Node(
        package="sim_pkg",
        executable="subscriber"
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(listener_node2)


    return ld