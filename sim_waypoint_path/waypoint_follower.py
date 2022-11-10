"""
A script that sends a robot to follow waypoints that form an equilateral polygon.
Attempts to use the bt_navigator node from the Nav2 framework, but I was unable to 
make it work.
"""

##
# file: navigate_waypoints.py
# date: 2022-11-10
# author: Channing Ko-Madden

import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sim_waypoint_path.basic_navigator import BasicNavigator
import sim_waypoint_path.pose_generators as pg

class WaypointFollower(Node):

    def __init__(self):
        super().__init__("waypoint_follower")
        self.initial_pose = None
        # declare and aquire "target_frame" parameter
        self.target_frame = self.declare_parameter(
                "target_frame", "base_link").get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._waitForTfPose()
        # create navigator
        self.nav = BasicNavigator() 
        self.nav.setInitialPose(self.initial_pose)
        self.nav.waitUntilNav2Active()


    def follow(self, sides, length):
        waypoint_poses = pg.create_polygon_pose(sides, length, self.initial_pose) 

        for pose in waypoint_poses:
            self.nav.goToPose(pose)

            while not self.nav.isNavComplete():
                feedback = self.nav.getFeedback()
                if feedback:
                    self.get_logger.debug("Estimated time of arrival: " + "{0.0f}".format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) 
                        + " seconds.")

            # wait for navigator to reach the pose
            result = self.nav.getResult()
            if result == GoalStatus.STATUS_SUCCEEDED:
                print("Pose goal succeeded!")

        print("All waypoints have been processed!")


    def _waitForTfPose(self):
        to_frame_rel = "map"
        while self.initial_pose is None:
            rclpy.spin_once(self)
            try:
                t = self.tf_buffer.lookup_transform(to_frame_rel, self.target_frame, rclpy.time.Time())
                self.initial_pose = Pose()
                self.initial_pose.orientation = t.transform.rotation
                self.initial_pose.position.x = t.transform.translation.x
                self.initial_pose.position.y = t.transform.translation.y
                self.initial_pose.position.z = t.transform.translation.z
            except Exception as ex:
                self.get_logger().info("Could not transform {to_frame_rel} to {self.target_frame}: {ex}")


def main(args=None):
    """
    Main function for running tool
    """
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Follow waypoint path using Nav2")
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
    #arguments = parser.parse_args(sys.argv[4:])
    arguments, unknown = parser.parse_known_args()

    follower = WaypointFollower()
    follower.follow(arguments.sides, arguments.length)


if __name__ == '__main__':
    main()
