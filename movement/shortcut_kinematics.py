# shortcut_kinematics.py
#
# This model solves the motion problem using a lazy shortcut, rather than
# leveraging all the fancy Jacobian-based kinematics models you might find in a
# textbook. Instead, this model leverages a convenient geometrical property of
# the xArm, letting us special-case the entire problem.
#
#  - The full motion neeeded to reach any destination is calculated in a single
#    step. By contrast, the jacobian kinematics and controller approach uses
#    multiple iterations to follow a kind of gradient towards the desired point.
#
#  - No attempt is made to move in a linear path. By contrast, the jacobian
#    kinematics and controller approach can make the robot move in a
#    nearly-linear path by follwing the steepest gradient.
#
#  - No attempt is made to move the end effector at a steady speed. All the
#    motors are moved at a rate so they (roughly) arrive at their target
#    positions simultaneously. Sometimes all the joint movements might
#    cumulatively add up to a high end effector velocity, other times the joint
#    movements might cancel out leading to slow end effector velocity.  By
#    contrast, the jacobian approach can allow for a controlled end effector
#    speed by normalizing each step of movement.
#
#  - This model leads to movement that always makes progress in joint space, but
#    not in cartesian space. So at times, the end effector might even move
#    *away* from the target point, though it will get there eventually. By
#    contrast, the jacobian approach will always follow the gradient in
#    cartesian space.
#
#  - This model never gets stuck at singularities, and will never try to 
#    exceed the joint limits. It can't, by design. It always makes a direct-line
#    path (in joint space), always staying within legal joint ranges. By
#    contrast, the jacobian can sometimes lead the arm towards "bad" positions,
#    where it hits a joint limit or gets trapped in a singularity.
#
#  - In essence, this is optimizing for travel time (by taking a straight line
#    in joint space), but not optimizing travel distance (in cartesian space).
#    By contrast, the jacobian approach optimizes for minimum travel distance in
#    cartesian space, at the expense of travel time. In a single path, the
#    jacobian approach might move a joint one way, then later move it back the
#    other way, so as to stay on a linear path. This model never does such a
#    thing, all motors move directly towards their final positions.
#
#  - Some target points have multiple solutions: the arm could bend "forwards"
#    to reach the point, or face 180 degrees opposite and bend over "backwards"
#    to reach the same point. This model always prefers to "bend forwards" if
#    that solution exists. It only resorts to bending backwards if there is no
#    other way to reach a point. By contrast, the jacobian just heads towards
#    whichever solution is closer (in cartesian space).

import numpy as np
import math
import random
from parameters import *
from direct_kinematics import *


pi = np.pi


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


def shortcut_solve(q_current, target, attack=None, verbose=False):
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

