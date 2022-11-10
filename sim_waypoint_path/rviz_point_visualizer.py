#! /usr/bin/env python3
"""
This script can be used to visualize a polygon waypoint path using RViz Markers.

To setup:

- Make sure RViz is up and running. By default there should be a global fixed frame
named "map". This is the frame that the markers will be published to.
- Within RViz, create/add a visualization by topic, which will list the
/visualization_marker topic as an option. 
- Run this script using: ros2 run sim_waypoint_path rviz_point_visualizer ...
- It is easiest to see the markers using the TopDownOrtho view within RViz
"""

##
# file: visualize_tools.py
# date: 2022-11-10
# author: Channing Ko-Madden

import threading
import argparse
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Pose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sim_waypoint_path.pose_generators import  create_polygon_pose

class RvizPointVisualizer(Node):
    """
    This class can be used to publish a list of Poses to RViz using the /visualization_marker topic
    """

    def __init__(self):
        super().__init__("rviz_point_visualizer")
        self.publisher = self.create_publisher(Marker, "visualization_marker", 10)


    def rviz_points(self, poses):
        """
        Send a list of poses as Markers to RViz for visualization

        Publishes the Markers in a separate thread.

        Parameters:
        -----------
        poses : list(Pose)
            The list of poses to visualize
        """
        publish_thread = threading.Thread(target=self._publish_thread, args=(poses,))
        publish_thread.start()


    def _publish_thread(self, poses):
        """
        Method for publishing the Marker messages, which can be run in a separate thread so that publishing
        does not block if the node has not been spun yet using rclpy.spin

        Assumes that RViz has a default frame named "map"

        Parameters:
        -----------
        poses : list(Pose)
            The list of poses to visualize
        """
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = super().get_clock().now().to_msg()
        marker.ns = "point_visuals"
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.id = 0
        marker.color.a = 1.0
        marker.color.r = 128.0

        # publish each pose as an individual marker
        for p in poses:
            marker.points.append(p.position)
            self.publisher.publish(marker)
            marker.id += 1
            marker.points.clear()
            self.get_logger().debug(p)


def main(args=None):
    """
    Main function for the running node tool
    """
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Visualize a polygon waypoint path with RViz")
    parser.add_argument(
            "-sides",
            "-s",
            type=int,
            help="The number of sides of the polygon (must be greater than 2)",
            required=True)
    parser.add_argument(
            "-length",
            "-l",
            type=float,
            help="The length of polygon sides (m) (must be greater than 0)",
            required=True)
    parser.add_argument(
            "-tf",
            "-t",
            action="store_true",
            help="Flag denoting to get initial pose from tf (assumes there is a 'base_link' and 'map' frame")
    arguments = parser.parse_args()

    visualizer = RvizPointVisualizer()
    
    start_pose = None
    if arguments.tf:
        to_frame_rel = "map"
        target_frame = "base_link"
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, visualizer)
        while start_pose is None:
            visualizer.get_logger().info("Attempting to get pose from tf")
            rclpy.spin_once(visualizer)
            try:
                t = tf_buffer.lookup_transform(to_frame_rel, target_frame, rclpy.time.Time())
                start_pose = Pose()
                start_pose.orientation = t.transform.rotation
                start_pose.position.x = t.transform.translation.x
                start_pose.position.y = t.transform.translation.y
                start_pose.position.z = t.transform.translation.z
            except Exception as ex:
               print(f"Could not transform {to_frame_rel} to {target_frame}: {ex}")
    else:
        start_pose = Pose()
        start_pose.orientation.w = 1.0
    visualizer.rviz_points(create_polygon_pose(arguments.sides, arguments.length, start_pose))
    visualizer.get_logger().info("Spinning node")
    rclpy.spin(visualizer)

    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
