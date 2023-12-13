from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    state_est_node = Node(
        package="drone_controls",
        executable="state_estimation",
    )

    control_node = Node(
        package="drone_controls",
        executable="command_drone",
    )

    ld.add_action(state_est_node)
    ld.add_action(control_node)

    return ld
