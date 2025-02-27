import numpy as np
import math
from parameters import *

pi = np.pi

cos_alpha = [ np.cos(alpha) for theta, alpha, a, d in dh_params ]
sin_alpha = [ np.sin(alpha) for theta, alpha, a, d in dh_params ]

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

def calculate_end_pos(q_current):
    """
    Direct Kinematics of the xArm Anthropomorphic Arm.
    Calculates end-effector position in real cartesian coordinates.
    """
    return anthroarm_dm(q_current)[0]

def anthroarm_diff(q):
    """
    Differential Kinematics for the xArm anthropomorphic arm.
    Calculates and returns the Jacobian matrix J.

    Parameters:
    q : array-like of joint angles [theta1, theta2, theta3, theta4, theta5, theta6].

    Returns:
    J : np.ndarray
        The Jacobian matrix relating joint velocities to end-effector velocities.
    """

    # Position of the end-effector
    # This is the first 3 elements of p = A01 * A12 * A23 * A34 * A45 * A56 * Origin
    # Where Origin is the vector [0, 0, 0, 1]'.
    p = anthroarm_dm(q)[0]

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

    # Link-end positions
    p0 = np.array([0, 0, 0])
    p1 = T01[:3, 3] # same as p01 = T01 x [0,0,0,1]' then taking first 3 coords
    p2 = T02[:3, 3] # same as p01 = T02 x [0,0,0,1]' then taking first 3 coords
    p3 = T03[:3, 3] # same as p01 = T03 x [0,0,0,1]' then taking first 3 coords
    p4 = T04[:3, 3] # same as p01 = T04 x [0,0,0,1]' then taking first 3 coords
    p5 = T05[:3, 3] # same as p01 = T05 x [0,0,0,1]' then taking first 3 coords

    # Rotation matrices
    R01 = A01[:3, :3]
    R12 = A12[:3, :3]
    R23 = A23[:3, :3]
    R34 = A34[:3, :3]
    R45 = A45[:3, :3]

    # Composed rotations
    R02 = R01 @ R12
    R03 = R02 @ R23
    R04 = R03 @ R34
    R05 = R04 @ R45

    # Joint axes
    z0 = np.array([0, 0, 1])
    z1 = R01 @ z0
    z2 = R02 @ z0
    z3 = R03 @ z0
    z4 = R04 @ z0
    z5 = R05 @ z0

    # Jacobian matrix
    J = np.vstack((
        np.column_stack((np.cross(z0, (p - p0)),
                         np.cross(z1, (p - p1)),
                         np.cross(z2, (p - p2)),
                         np.cross(z3, (p - p3)),
                         np.cross(z4, (p - p4)),
                         np.cross(z5, (p - p5)))),
        np.column_stack((z0, z1, z2, z3, z4, z5))
    ))

    return J

def cartesianToString(pos):
    return "[ x: %4.1f mm, y: %4.1f mm, z: %4.1f mm ]" % ( pos[0]*1000, pos[1]*1000, pos[2]*1000 )

def calculate_joint_angles_delta(q_current, target, step_size=0.175):
    """
    Calculate the joint angle changes required for a desired end-effector change.
    
    Parameters:
    q_current : array-like
        Current joint angles [theta1, theta2, theta3, theta4, theta5, theta6].
    target : array-like
        Desired end-effector position. Does not include orientation.
        This should be a 3-element array: [x, y, z]
        Ideally this would include orientation, with [roll, pitch, yaw] as well.
    
    Returns:
    delta_q : np.ndarray
        The change in joint angles to achieve the desired end-effector change.
    """

    # Before anything else, calculate the optimal base rotation angle.
    # This can be solved directly:
    # - using target x, y, z, calculate the target angle
    # - if within base min/max angle, then use that as our target angle
    # - otherwise, we need to bend over backwards, so use the opposite angle
    # - examine the current base angle
    # - if within step_size radians, then base should just move that amount
    # - otherwise base should move step_size radians in correct direction
    targetAngle = math.atan2(target[1], target[0])
    if targetAngle < joint_min_rad[0]:
        targetAngle += pi
    elif joint_max_rad[0] < targetAngle:
        targetAngle -= pi
    baseAngle = q_current[0]
    baseDelta = targetAngle - baseAngle # np.clip(targetAngle - baseAngle, -step_size, step_size)
    # print("target base angle %.3f deg, current base %.3f deg, move %.3f deg" % (
    #     np.degrees(targetAngle), np.degrees(baseAngle), np.degrees(baseDelta)))
    # Before rest of calculations, modify q_current to reflect where the
    # base will eventually be rotated to.
    # q_current = q_current.copy()
    # q_current[0] = targetAngle

    # Compute the current position
    end_pos = anthroarm_dm(q_current)[0]
    # print("end_pos: " + cartesianToString(end_pos))

    # Compute the Jacobian matrix at the current joint angles
    J = anthroarm_diff(q_current)
    # J[:,4] = np.transpose(np.array([0,0,0,0,0,0]))
    J = J[0:3,:]
    
    # Compute the pseudo-inverse of the Jacobian
    J_pinv = np.linalg.pinv(J)
   
    err = np.linalg.norm(target - end_pos)
    # print(f"Current error: {err*1000} mm from target")
    # If error is above 30 mm (0.030 m), we want to move by the full step_size
    # in radians. As error falls below 30 mm, and approaches zero, we want
    # to decrease the movement amount. So:
    #  when err < 0.030 then adjust by sqrt(err/0.030)
    #  when err >= 0.030 then adjust by 1.0
    step_size_adjusted = step_size * min(1.0, math.sqrt(err/0.030))
    q_delta = J_pinv @ (target - end_pos)

    # Fix q_delta so base rotates optimally
    q_delta[0] = baseDelta
    # Normalize q_delta so we move approx step_size_adjusted radians total across all joints
    q_delta = (q_delta / np.linalg.norm(q_delta)) * step_size_adjusted

    # print(f"err = %.2f mm from target position" % (err*1000))
    # print("step size = approx %.3f degrees total across all servos" % (step_size_adjusted * 180/pi))
    # print("q_delta = [ %6.2f°  %6.2f°  %6.2f°  %6.2f°  %6.2f°  %6.2f° ]" % (
    #     q_delta[0]*180/pi, q_delta[1]*180/pi, q_delta[2]*180/pi,
    #     q_delta[3]*180/pi, q_delta[4]*180/pi, q_delta[5]*180/pi))
    
    return q_delta, err

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

