"""
pytest Tests for the pose_generators module

To run the test, from the base of the repo run: python3 -m pytest test/test_pose_generators.py

"""

##
# file: test_pose_generators.py
# date: 2022-11-09
# author: Channing Ko-Madden

import math

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
            Each side of the polygon should be the same correct length
            The yaw difference between each pose should be correct and the same for each pose.
            The last pose should be in the same x,y position as the start pose
        """
        side_length_m = 5.0 
        start_pose = Pose()
        start_pose.orientation.w = 1.0

        # test creating polygon of multiple sides
        for i in range(3, 9):
            poses = pg.create_polygon_pose(i, side_length_m, start_pose)
            assert(len(poses) == i) # check there are the correct number of poses
            # check that the final pose has the same x,y position as the start
            assert(poses[-1].position.x == pytest.approx(start_pose.position.x))
            assert(poses[-1].position.y == pytest.approx(start_pose.position.y))

            rot_angle_rad = math.pi - pg.regular_polygon_interior_angle(i)
            for i in range(0, len(poses) - 1):
                # calculate the yaw change between orientations to confirm they are correct
                inverse_q = poses[i].orientation
                inverse_q.w *= -1 # negate for inverse quaternion

                rot_quat = pg.quaternion_multiply(poses[i+1].orientation, inverse_q)
                rot_euler = pg.euler_from_quaterion(rot_quat)
                # assert the magnitude of rotation (regardless of direction) between poses is correct
                assert(abs(rot_euler.z) == pytest.approx(rot_angle_rad)) 

                # calculate the euclidian distance between poses to ensure they are the correct size
                length = math.sqrt(math.pow(poses[i].position.x - poses[i+1].position.x, 2)
                    + math.pow(poses[i].position.y - poses[i+1].position.y, 2))
                assert(length == pytest.approx(side_length_m))
