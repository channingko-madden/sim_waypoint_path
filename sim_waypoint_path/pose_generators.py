"""
This module contains classes and functions for generating poses, as well
as tools for working with quaternions

"""

##
# file: pose_generators.py
# date: 2022-11-10
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


def quaternion_multiply(quat0, quat1):
    """
    Multiplies two Quaternions and returns the resulting Quaternion

    Parameters:
    -----------
    quat0: Quaternion 
        Quaternion msg object
    quat1: Quaternion 
        Quaternion msg object
    """

    output = Quaternion()
    output.w = quat0.w * quat1.w - quat0.x * quat1.x - quat0.y * quat1.y - quat0.z * quat1.z
    output.x = quat0.w * quat1.x + quat0.x * quat1.w + quat0.y * quat1.z - quat0.z * quat1.y
    output.y = quat0.w * quat1.y - quat0.x * quat1.z + quat0.y * quat1.w + quat0.z * quat1.x
    output.z = quat0.w * quat1.z + quat0.x * quat1.y - quat0.y * quat1.x + quat0.z * quat1.w

    return output


def regular_polygon_interior_angle(sides):
    """
    Return the interior angle for a regular polygon with a given number of sides,
    in radians

    A regular polygon is defined as a polygon that is equiangular and equilateral

    Parameters:
    -----------
    sides : int
        The number of sides of the polygon (should be greater than 2)

    """
    return ((sides - 2) * math.pi) / sides

def create_polygon_pose(sides, length, current_pose):
    """
    Return a list of 2D poses that form an equilateral polygon. The list of poses are
    in "clockwise" order.  
    Will return an empty list if the values for sides or length are improper.

    Parameters:
    -----------
    sides : int
        The number of sides of the polygon (must be greater than 2)
    length : float
        The length of the sides of the polygon in meters (must be greater than 0)
    current_pose : The current 2D pose of the robot
        Pose must contain a valid orientation
    """

    # Cannot have a polygon with less than 3 sides or length less than 0!
    if sides <= 2 or length <= 0:
        return []

    # calculate the yaw change between the poses within the polygon
    # Yaw = 180 - interior angle of polygon
    rot_angle_rad = math.pi - regular_polygon_interior_angle(sides)

    ret_poses = [] # the list that is returned
    
    # Default to have the first pose always be directly in front of the
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
        current_euler.z = current_euler.z - rot_angle_rad # turning clockwise so subtract
        next_pose.orientation = quaternion_from_euler(current_euler)
        next_pose.position.x = ret_poses[-1].position.x + length * math.cos(current_euler.z)
        next_pose.position.y = ret_poses[-1].position.y + length * math.sin(current_euler.z)
        ret_poses.append(next_pose)

    return ret_poses
