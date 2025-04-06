# direct_kinematics.py
#
# This code implements forward-kinematics and some helper functions to work with
# cartesian-space and joint-space coordinate systems.
#
# These functions are mostly used as helpers by the higher-level robot code
# (e.g. to check if a target point is within reach of the robot arm), or by the
# kinematics code (e.g. to calculate the end-effector position for a given set
# of motor angles).

import numpy as np
import math
import random
from parameters import *

pi = np.pi

cos_alpha = [ np.cos(alpha) for theta, alpha, a, d in dh_params ]
sin_alpha = [ np.sin(alpha) for theta, alpha, a, d in dh_params ]


def calculate_end_pos(q_current):
    """
    Direct Kinematics of the xArm Anthropomorphic Arm.
    Calculates end-effector position in real cartesian coordinates.
    """
    return anthroarm_dm(q_current)[0]


def nearest_reachable_point(target):
    """
    Compute a point that is within reach and close to a target position.

    Parameters:
    target : array-like
        Desired end-effector position.
        This should be a 3-element array: [x, y, z]

    Returns:
    nearby: np.ndarray
        An [x, y, z] position near target that is within reach of the arm.
    moved: boolean
        Whether the target was out of reach.
    """

    pos = np.array(target) # vector from origin to target
    link0 = np.array([0, 0, d1]) # vector from origin to shoulder
    pos = pos - link0 # vector from shoulder to target
    max_reach = a2 + a3 + d5
    target_dist = np.linalg.norm(pos)
    if target_dist > max_reach: # adjust target to be closer to shoulder
        pos = pos * (max_reach/target_dist)
        nearby = link0 + pos
        return nearby, True
    else:
        return target, False # no adjustment needed


def xform_matrix(i, theta_i):
    """
    Returns the 4x4 transform matrix for joint i.
    Parameters:
      i = integer from 1 to 5
      theta_i = current angle of joint i, in radians, where typically
                joint_min_rad[i-1] <= theta_i <= joint_max_rad[i-1]

      returns: np.ndarray - 4x4 transformation matrix
    """
    offset, alpha, a, d = dh_params[i-1]
    theta = offset + theta_i

    cosT = np.cos(theta)
    sinT = np.sin(theta)
    cosA = cos_alpha[i-1]
    sinA = sin_alpha[i-1]

    return np.array([ [cosT, -sinT*cosA,  sinT*sinA, a*cosT],
                      [sinT,  cosT*cosA, -cosT*sinA, a*sinT],
                      [   0,       sinA,       cosA,      d],
                      [   0,          0,          0,      1] ])


def anthroarm_dm(q):
    """
    Direct Kinematics of the xArm Anthropomorphic Arm.
    Calculates end-effector position and orientation in real cartesian coordinates.

    Parameters:
    q : array-like of joint angles [theta1, theta2, theta3, theta4, theta5, theta6] in radians.

    Returns:
    p : np.ndarray, Position [ x, y, z ] of the end-effector in 3D space.
    a : np.ndarray, Orientation vector aligned with the direction of the final link.
    s : np.ndarray, Orientation vector orthogonal to a and aligned with the width of the grabber.
    n : np.ndarray, Orientation vector orthogonal to both a and s.
    """
    
    # Homogeneous transformation matrices
    A01 = xform_matrix(1, q[0])
    A12 = xform_matrix(2, q[1])
    A23 = xform_matrix(3, q[2])
    A34 = xform_matrix(4, q[3])
    A45 = xform_matrix(5, q[4])

    # Composed transformations
    T01 = A01
    T02 = T01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34
    T05 = T04 @ A45

    n = T05 @ np.array([1, 0, 0, 1])
    s = T05 @ np.array([0, 1, 0, 1])
    a = T05 @ np.array([0, 0, 1, 1])
    p = T05 @ np.array([0, 0, 0, 1])

    return p[:3], a[:3], s[:3], n[:3]


def cartesianToString(pos):
    return "[ x: %4.1f mm, y: %4.1f mm, z: %4.1f mm ]" % ( pos[0]*1000, pos[1]*1000, pos[2]*1000 )

