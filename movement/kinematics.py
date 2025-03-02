import numpy as np
import math
import random
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

def golden_grid_search(func, x_low, x_high, tol=1e-5, grid_steps=8, max_iter=1000):
    """
    Find minimum using a combination of grid and golden section search.

    Parameters:
    func -- the function to minimize
    x_low, x_high -- bounds of the search interval
    tol -- tolerance for stopping criterion
    grid_steps -- number of grid points to check initially
    max_iter -- maximum number of iterations
    """
    
    xd = x_high - x_low
    def x_i(i):
        return x_low + xd * (i / grid_steps)
    x, y = (x_low + x_high)/2, math.inf
    for i in range(grid_steps):
        x_min, y_min = golden_search(func, x_i(i), x_i(i+1), tol, max_iter)
        # print(f"AoA from {x_i(i)} to {x_i(i+1)}, best is f({x_min}) = {y_min}")
        if y_min < y:
            x, y = x_min, y_min
    return x, y

def golden_search(func, x_low, x_high, tol=1e-3, max_iter=100):
    """
    Find minimum using golden section search.

    Parameters:
    func -- the function to minimize
    x_low, x_high -- bounds of the search interval
    tol -- tolerance for stopping criterion
    max_iter -- maximum number of iterations

    Returns:
    x_min -- x value that minimizes func
    y_min -- minimum value of func
    """

    golden_ratio = (np.sqrt(5) - 1) / 2  # ≈ 0.618

    # Initialize intermediate points
    x1 = x_high - golden_ratio * (x_high - x_low)
    x2 = x_low + golden_ratio * (x_high - x_low)
    f1 = func(x1)
    f2 = func(x2)

    iteration = 0

    while abs(x_high - x_low) > tol and iteration < max_iter:
        iteration += 1

        if f1 > f2 or (f1==f2 and bool(random.getrandbits(1))):  # Minimum is in the right interval
            x_low = x1
            x1 = x2
            f1 = f2
            x2 = x_low + golden_ratio * (x_high - x_low)
            f2 = func(x2)
        else:  # Minimum is in the left interval
            x_high = x2
            x2 = x1
            f2 = f1
            x1 = x_high - golden_ratio * (x_high - x_low)
            f1 = func(x1)

    # Find best point
    x_min = (x_low + x_high) / 2
    y_min = func(x_min)

    return x_min, y_min

def circle_intersection_points(x1, y1, r1, x2, y2, r2):
    """
    Calculate the intersection points of two circles, or the nearest such point if there is no intersection

    Args:
        x1, y1: Center coordinates of the first circle
        r1: Radius of the first circle
        x2, y2: Center coordinates of the second circle
        r2: Radius of the second circle

    Returns:
        List of intersection points (may contain one point, or two points), or a list with one point
        containing a nearest near-intersecting point. Also returns boolean to indicate if valid.
    """
    # Calculate the distance between the centers
    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Check if circles don't intersect
    if d > r1 + r2: # too distant to intersect
        gap = d - (r1 + r2)
        f = (r1 + gap/2)/d
        x, y = x1 + f*(x2 - x1), y1 + f*(y2 - y1)
        return ([(x, y)], False)
    elif d < abs(r1 - r2): # one circle inside the other
        f = 0.5 + (r1 + r2)/(2*d)
        x, y = x1 + f*(x2 - x1), y1 + f*(y2 - y1)
        return ([(x, y)], False)

    # Check if circles are tangent (one intersection point)
    if math.isclose(d, r1 + r2) or math.isclose(d, abs(r1 - r2)):
        # Calculate the single intersection point
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        x = x1 + a * (x2 - x1) / d
        y = y1 + a * (y2 - y1) / d
        return ([(x, y)], True)

    # Calculate the general case (two intersection points)
    # Distance from center1 to the line connecting intersection points
    a = (r1**2 - r2**2 + d**2) / (2 * d)

    # Height from the line to intersection points
    h = math.sqrt(r1**2 - a**2)

    # Point on the line connecting centers
    x_p2 = x1 + a * (x2 - x1) / d
    y_p2 = y1 + a * (y2 - y1) / d

    # Calculate both intersection points
    x3 = x_p2 + h * (y2 - y1) / d
    y3 = y_p2 - h * (x2 - x1) / d

    x4 = x_p2 - h * (y2 - y1) / d
    y4 = y_p2 + h * (x2 - x1) / d

    return ([(x3, y3), (x4, y4)], True)

def direct_solve(q_current, target, attack=None, verbose=False):
    """ Directly compute the joint angles corresponding to a given cartesian target.

    This doesn't use the jacobian or DH formulation at all, and it makes no
    attempt at straight-line motion. Instead, it takes advantage of specific
    limitations in the arm to directly compute the final target angles that
    match the cartesian x, y, z coordinates.

    If an angle of attack is provided as a float, or a (low, high) range, then
    only that angle or range of angles will be considered. As a parameter, an
    attack angle is defined as:
      0 is parallel to the table stretching outward away from the origin,
      pi/2 is stretching up towards the sky,
      -pi/2 has the the gripper is pointing down,
      pi, or -pi, has the gripper pointing towards the origin,
      and so on.

    For the base rotation, there is a closed form solution:
    * As a special case, if (x, y) is (0, 0), then we leave the base angle
      alone, since any base angle will work. Alternatively, we could move to a
      resting base angle of 0.
    * Otherwise, if (x, y) is off the origin, there are only two base angles
      that position the arm to move within a plane passing through (x, y). These
      angles are opposite each other. If only one is a legal base angle, we
      choose that one. Otherwise, we choose whichever one puts (x, y) on the
      "forward" side, so the arm doesn't need to bend over backwards. 

    After the base rotation, the problem is reduced to 2D plane geometry. But
    there are two degrees of freedom that can lead to multiple solutions for the
    shoulder, elbow, and wrist angles. We do iterative optimization to solve
    this numerically.
    * First, is the angle of attack: we need to consider all possible angles at
      which the grabber might be oriented towards the target point, e.g. grabber
      oriented down to the target from above, or up from from below, or grabber
      positioned to the side or wfrom some angle.
    * Second, given some attack angle, we can geometrically solve for the
      shoulder and elbow angles. There maybe two solutions to these equations,
      corresponding to an "elbow convex" or "elbow concave" orientation:

                      ._-x                        x
                   o-'                            |
                   |  convex                     ,o   concave
                   |  "up"                    ,-'     "down"
                   O                         O
    * Given the shoulder and elbow angles, and the attack angle, the wrist angle
      can be calculated directly.
    Given these degrees of freedom, we always choose convex, if available, and
    from among all possible convex solutions, we choose an attack angle such
    that the three joint angles are closest to their resting 0 points, e.g.
    minimizing the sum of squares of the angles. 
    """
   
    tx, ty, tz = target
    
    # Within plane of movement, calculate target position.
    px, py = ((tx**2) + (ty**2))**0.5, tz
    # Also, shoulder joint within plane.
    sx, sy = 0, d1

    # First, calculate optimal base rotation angle. This is solved directly.
    backwards = False
    if tx == 0 and ty == 0:
        a_b = q_current[0] # if origin, leave the base angle alone
    else:
        a_b = math.atan2(ty, tx) # else, find plane passing through x, y
        if a_b < joint_min_rad[0]:
            a_b += pi
            backwards = True
        elif joint_max_rad[0] < a_b:
            a_b -= pi
            backwards = True

    if backwards:
        px = -px

    if verbose:
        print(f"Target point {px} {py}")
        print(f"Shoulder point {sx} {sy}")

    # Note: in all of the below, attack angle is direction the gripper points,
    # measured counter-clockwise from the x axis within the plane of movement.

    # Function to calculate joint angles for a given angle of attack, and boolean to indicate validity
    def joint_angles(a, verbose=False):
        # Wrist joint within plane.
        wx, wy = px - d5 * np.cos(a), py - d5 * np.sin(a)
        # Find possible elbow joint points
        pts, valid = circle_intersection_points(sx, sy, a2, wx, wy, a3)
        # print(pts, valid)
        if verbose:
            print(f"AoA {np.degrees(a)} wrist is at {wx}, {wy}, elbow pts are {pts} (valid = {valid})")
        # prioritize whichever elbow point is higher above the table surface
        candidates = pts if len(pts) == 1 or pts[0][1] >= pts[1][1] else [pts[1], pts[0]]
        alt = None
        for ex, ey in candidates:
            # calculate shoulder joint angle
            x_se, y_se = ex - sx, ey - sy
            a_s = math.atan2(-x_se, y_se) # anti-clockwise from y axis
            if verbose:
                print(f"se is {x_se}, {y_se}, angle {np.degrees(a_s)}")
            # calculate elbow joint angle
            x_ew, y_ew = wx - ex, wy - ey
            a_e_x = math.atan2(-y_ew, x_ew) # clockwise from x axis
            a_e = a_e_x + a_s + pi/2 # clockwise from shoulder-elbow axis
            if verbose:
                print(f"ew is {x_ew}, {y_ew}, angle {np.degrees(a_e)} = {np.degrees(a_e_x)} + {np.degrees(a_s)} + {pi/2}")
            if a_e > pi:
                a_e -= 2*pi
            elif a_e < -pi:
                a_e += 2*pi
            # calculate wrist joint angle
            x_wp, y_wp = px - wx, py - wy
            a_w_x = math.atan2(y_wp, x_wp) # anti-clockwise from x axis
            a_w = a_w_x + a_e_x # anti-clockwise from elbow-wrist axis
            if verbose:
                print(f"wp is {x_wp}, {y_wp}, angle {np.degrees(a_w)} = {np.degrees(a_w_x)} + {np.degrees(a_e_x)}")
            if a_w > pi:
                a_w -= 2*pi
            elif a_w < -pi:
                a_w += 2*pi
            # check limits
            if verbose:
                print(f"checking {np.degrees(a_s)}, {np.degrees(a_e)}, {np.degrees(a_w)} degrees for limits")
            alt = [ a_s, a_e, a_w ]
            if a_s < joint_min_rad[1] or joint_max_rad[1] < a_s:
                continue
            if a_e < joint_min_rad[2] or joint_max_rad[2] < a_e:
                continue
            if a_w < joint_min_rad[3] or joint_max_rad[3] < a_w:
                continue
            return [a_s, a_e, a_w], valid
        # The candidates can't be reached within joint limits, so just do the best we can
        a_s = np.clip(alt[0], joint_min_rad[1], joint_max_rad[1])
        a_e = np.clip(alt[1], joint_min_rad[2], joint_max_rad[2])
        a_w = np.clip(alt[2], joint_min_rad[3], joint_max_rad[3])
        return [a_s, a_e, a_w], False

    def position_error(angles):
        a_s, a_e, a_w = angles
        q = np.array([ a_b, a_s, a_e, a_w, 0, 0 ])
        end_pos = anthroarm_dm(q)[0]
        err = np.linalg.norm(target - end_pos)
        return err

    def fitness(a): # fitness evaluation for given angle of attack
        angles, valid = joint_angles(a)
        if valid:
            err = sum([a**2 for a in angles]) if angles else math.inf
            # print(f"AoA {a} --> err {err} for valid angles {angles}")
        else:
            penalty = position_error(angles)
            err = penalty**2 + 3*(pi)**2 + sum([a**2 for a in angles]) if angles else math.inf
            # print(f"AoA {a} --> penalty {penalty*1000} mm distance, err {err} for invalid angles {angles}")
        return err

    # Note: here we need to adjust the angle of attack so it is relative to the
    # plane of movement, which might be backwards.
    if attack is None:
        attack = (-pi, pi)
    elif backwards and (isinstance(attack, tuple) or isinstance(attack, list)):
        attack = (attack[0] + pi, attack[1] + pi)
    elif backwards:
        attack = attack + pi

    if isinstance(attack, tuple) or isinstance(attack, list):
        # Next, search all angles of attack, optimize fitness.
        a, _ = golden_grid_search(fitness, attack[0], attack[1])
        if verbose:
            print(f"Best angle of attack is {np.degrees(a)} degrees")
    else:
        a = attack
    
    # Calculate angles
    angles, valid = joint_angles(a, verbose=False)
    if not angles:
        # This target position is not reachable.
        return None
    return [a_b] + angles

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
    pass


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

