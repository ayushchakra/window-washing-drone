from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    [
                        "ros2 run apriltag_ros apriltag_node --ros-args "
                        "-r image_rect:=/drone/bottom/image_raw -r "
                        "camera_info:=/drone/bottom/camera_info --params-file "
                        "`ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml"
                    ]
                ],
                shell=True,
            ),
        ]
    )
