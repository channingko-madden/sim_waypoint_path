"""
Tests for the pose_generators module

"""

##
# file: test_pose_generators.py
# date: 2022-11-07
# author: Channing Ko-Madden

import pytest

from geometry_msgs.msg import Vector3, Pose

from sim_waypoint_path import pose_generators as pg


class TestPoseGenerators:
    """
    pytest Class for testing the pose_generators module
    """

    def test_euler_quaterion(self):
        """
        Test the conversion functions for euler to quaternion, and
        quaternion to euler
        """
        euler = Vector3()
        euler.x = 0.5
        euler.y = 0.75
        euler.z = 1.0
        quat = pg.quaternion_from_euler(euler)
        new_euler = pg.euler_from_quaterion(quat)

        assert(euler.x == pytest.approx(new_euler.x))
        assert(euler.y == pytest.approx(new_euler.y))
        assert(euler.z == pytest.approx(new_euler.z))

    def test_create_polygon_pose(self):
        """
        Test the create_polygon_pose function.
        Each side of the polygon should be the same correct length, and
        the heading difference between each pose should be correct and the
        same for each pose
        """
        poses = pg.create_polygon_pose(4, 5.0)
        assert(len(poses) == 4)
        # check that the final pose has the same x,y position as the start
        assert(poses[-1].position.x == pytest.approx(0.0))
        assert(poses[-1].position.y == pytest.approx(0.0))
        print(poses)

        start_pose = Pose()
        start_pose.position.x = 5.0
        start_pose.position.y = 2.5

        poses = pg.create_polygon_pose(4, 5.0, start_pose)
        assert(len(poses) == 4)
        # check that the final pose has the same x,y position as the start
        assert(poses[-1].position.x == pytest.approx(5.0))
        assert(poses[-1].position.y == pytest.approx(2.5))
        print(poses)


def print_pose_list():
        poses = pg.create_polygon_pose(4, 5.0)
        print(poses)


if __name__ == "__main__":
    print_pose_list()
