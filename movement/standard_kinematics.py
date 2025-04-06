# standard_kinematics.py
#
# This model solves the motion problem using a standard "textbook" approach,
# using a Jacobian and an iterative controller. Given a target position in
# cartesian space, this code will calculate what rate and direction to move each
# motor to head towards that target in the most direct (cartesian) line, i.e.
# following a gradient of fastest descent to reduce the (cartesian) distance to
# the target.
#
#  - Tihs code needs to be coupled with an iterative motion controller, or a
#    some other kind of path planner, because the vector calculated is valid
#    only "instantaneously". As soon as the arm begins to move, the vector must
#    be recalculated.
#
#  - This will (ideally) lead to a nearly-linear path in cartesian space.
#
#  - This can move the end-effector at a steady, controlled velocity (in
#    cartesian space). In principle, the vector can be normalized to achieve a
#    constant velocity, or adjusted to ramp the acceleration. The code below
#    kind of does this?
#
#  - This model (ideally) will make steady progress in cartesian space.
#
#  - This model can get stuck at singularities, and may try to exceed the joint
#    limits. The controller using this model needs to check for these things.
#
#  - In essence, this is optimizing for travel distance (by taking a straight line
#    in cartesian space), but not optimizing travel time (in joint space). In a
#    single path, the controller might move a joint one way, then later move it
#    back the other way, so as to stay on a linear path. 
#
#  - Some target points have multiple solutions: the arm could bend "forwards"
#    to reach the point, or face 180 degrees opposite and bend over "backwards"
#    to reach the same point. This model just heads towards whichever solution
#    is closer (in cartesian space), even if it means we end up bending over
#    backwards in some cases where a forward-bend would look nicer.

import numpy as np
import math
import random
from parameters import *
from direct_kinematics import *


pi = np.pi


def plan_path(q_current, target, step_size=0.175, verbose=False):
    """ This is similar to calculate_joint_angles_delta, but it does some extra
    planning.

    It returns a sequence of It returns a list of joint angle vectors, q_0, q_1,
    q_2, ..., q_N, forming a path to reach the target.

      * Each q_i is a legal configuration of joint angles, within the joint
        limits.
      * Each q_(i+i) is reachable from q_i directly by a legal and small
        movement of the joints, and similarly q_0 is reachable from the current
        position q_current directly by a legal and small movement of the joints.
      * q_N is at the target, within some reasonable margin of error.
      * The overall path from q_current, to q_0, to q_1, ..., to q_N, is
        reasonably short.

    The idea is the that we can just call set_position_radians(qi) for i=0 to N,
    with some suitable delay between each to allow time for movement.
    """
    pass # todo... maybe?


def calculate_joint_angles_delta(q_current, target, step_size=0.175, verbose=False):
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
    if verbose:
        print(f"Current error: {err*1000} mm from target")
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

    if verbose:
        print(f"err = %.2f mm from target position" % (err*1000))
        print("request step size = %.3f degrees total across all servos" % (step_size * 180/pi))
        print("adjust step size = approx %.3f degrees total across all servos" % (step_size_adjusted * 180/pi))
        print("q_delta = [ %6.2f°  %6.2f°  %6.2f°  %6.2f°  %6.2f°  %6.2f° ]" % (
            q_delta[0]*180/pi, q_delta[1]*180/pi, q_delta[2]*180/pi,
            q_delta[3]*180/pi, q_delta[4]*180/pi, q_delta[5]*180/pi))
    
    return q_delta, err


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

