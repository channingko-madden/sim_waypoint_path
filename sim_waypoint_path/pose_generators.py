"""
Classes and functions for generating poses

"""

##
# file: pose_generators.py
# date: 2022-11-07
# author: Channing Ko-Madden

import math

from geometry_msgs.msg import Pose, Quaternion, Vector3


def euler_from_quaterion(quat):
    """
    Convert a Quaternion msg object into a Vector3 msg that contains the equivalent
    roll, pitch, and yaw [x, y, z] in radians.

    For background see:
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    Parameters:
    -----------
    quat : Quaternion
        The quaternion to convert
    """

    euler = Vector3()
    sinr_cosp = 2.0 * (quat.w * quat.x + quat.y * quat.z)
    cosr_cosp = 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
    euler.x = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x)
    if abs(sinp) >= 1.0:
        euler.y = math.copysign(math.pi / 2, sinp)
    else:
        euler.y = math.asin(sinp)

    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    euler.z = math.atan2(siny_cosp, cosy_cosp)

    return euler


def quaternion_from_euler(euler):
    """
    Convert a Vector3 containing euler angles (in radians) into a Quaternion msg object

    For background see:
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    Parameters:
    -----------
    euler : Vector3
        x -> roll, y -> pitch, z -> yaw
    """

    cr = math.cos(euler.x * 0.5)
    sr = math.sin(euler.x * 0.5)
    cp = math.cos(euler.y * 0.5)
    sp = math.sin(euler.y * 0.5)
    cy = math.cos(euler.z * 0.5)
    sy = math.sin(euler.z * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def create_polygon_pose(sides, length, current_pose=Pose()):
    """
    Return a list of 2D poses that form an equilateral polygon. The list of poses will
    in "clockwise" order (meaning robot will travel the polygon clockwise).

    Parameters:
    -----------
    sides : int
        The number of sides of the polygon (must be greater than 2)
    length : float
        The length of the sides of the polygon in meters (must be greater than 0)
    current_pose : The current 2D pose of the robot
    """

    # Cannot have a polygon with less than 3 sides or length less than 0!
    if sides <= 2 or length <= 0:
        return []

    # calculate the angle of rotation between the poses within the polygon
    rot_angle_rad = ((sides - 2) * math.pi) / sides

    ret_poses = []
    # Lets default to have the first pose always be directly in front of the
    # robots current heading
    current_euler = euler_from_quaterion(current_pose.orientation)
    next_pose = Pose()
    next_pose.orientation = current_pose.orientation
    next_pose.position.z = current_pose.position.z
    next_pose.position.x = current_pose.position.x + length * math.cos(current_euler.z)
    next_pose.position.y = current_pose.position.y + length * math.sin(current_euler.z)
    ret_poses.append(next_pose)

    for i in range(0, sides - 1):
        next_pose = Pose()
        current_euler.z = current_euler.z - rot_angle_rad
        next_pose.orientation = quaternion_from_euler(current_euler)
        next_pose.position.x = ret_poses[-1].position.x + length * math.cos(current_euler.z)
        next_pose.position.y = ret_poses[-1].position.y + length * math.sin(current_euler.z)
        ret_poses.append(next_pose)

    return ret_poses
