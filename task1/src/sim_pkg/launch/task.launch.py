from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    simulator_node = Node(
        package="sim_pkg",
        executable="simulator",
    )
    listener_node = Node(
        package="sim_pkg",
        executable="sender"
    )
    subscriber_node = Node(
        package="sim_pkg",
        executable="subscriber"
    )

    ld.add_action(simulator_node)
    ld.add_action(listener_node)
    ld.add_action(subscriber_node)


    return ld