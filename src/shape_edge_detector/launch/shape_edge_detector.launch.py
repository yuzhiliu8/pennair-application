from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # shape_edge_detector_pkg_dir = get_package_share_directory("shape_edge_detector")
    # video_publisher_pkg_dir = get_package_share_directory("video_publisher")

    # shape_edge_detecotor_params = os.path.join(shape_edge_detector_pkg_dir, 'config', 'shape_edge_detector_params.yaml')

    video_publisher_node = Node(
        package="video_publisher",
        executable="publisher",
        name="video_publisher"
    )

    shape_edge_detector_node = Node(
        package="shape_edge_detector",
        executable="shape_edge_detector",
        name="shape_edge_detector",
    )


    return LaunchDescription([
        video_publisher_node,
        shape_edge_detector_node
    ])

    

