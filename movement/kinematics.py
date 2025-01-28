import numpy as np
import math

pi = np.pi
pi3 = 3*np.pi

# xArm Morphological Parameters and Limits

#                       ---Clicks----    ---------Radians-----------   ----------Degrees---------
#  Joint  Servo/ID      Min  Ctr  Max      Min      Ctr      Max         Min       Ctr        Max
#    6    grip/1        160  432  704    -pi/2       0      +pi/2        -90        0          90
#    5    hand/2        128  488  848    -pi         0      +pi         -180        0         180
#    4    wrist/3        58  492  927    -3pi/4      0      +3pi/4      -135        0         135
#    3    elbow/4         4  498  993    -3pi/4      0      +3pi/4      -135        0         135
#    2    shoulder/5    144  512  880    -pi/2       0      +pi/2        -90        0          90
#    1    base/6          0  504 1008    -pi         0      +pi         -180        0         180
# NOTE: These are from a mix of sources and should be verified against the
# physical arm. In particular, the ranges are not actually symmetric at all.

#                   base   shldr   elbow   wrist    hand    grip
#         joint        1       2       3       4       5       6
joint_min_clk = [      0,    144,      4,     58,    128,    160 ] # min extent, in clicks
joint_max_clk = [   1008,    880,    992,    926,    848,    704 ] # max extent, in clicks
joint_min_rad = [    -pi,  -pi/2,  -pi/2,  -pi/2,    -pi,  -pi/2 ] # min extent, in radians
joint_max_rad = [     pi,   pi/2,   pi/2,   pi/2,     pi,   pi/2 ] # max extent, in radians

#                                    _           , . ,    <-- imaginary end point
#         grip finger length .......|            \   /
#                 +                 |_            \_/     <-- (Servo 1) Grip joint, open/close
# d5 =      hand-grip length .......|             /_\
#                 +                 |_            |=|     <-- (Servo 2) Hand joint, rotation
#          wrist-hand length .......|             | |
#                                   |_            (o)     <-- (Servo 3) Wrist joint, bending
#                                   |             | |
# a3 =    elbow-wrist length .......|             | |
#                                   |_            (o)     <-- (Servo 4) Elbow joint, bending
#                                   |            / /
# a2 = shoulder-elbow length .......|           / /
#                                   |          / /
#                                   |_        (o)         <-- (Servo 5) Shoulder joint, bending
# d1 =           base height .......|      ___|=|___ 
#                                   |_   /_____._____\    <-- (Servo 6) Base joint, rotation
#                                              ^---Origin of cartesian coordinate system,
#                                                  where x axis is to the right,
#                                                        y axis is into the page, and
#                                                        z axis is vertical.

d1 = 0.100  # meters, height of shoulder joint above table or baseplate
a2 = 0.155  # meters, distance between shoulder joint and elbow joint
a3 = 0.096  # meters, distance between elbow joint and wrist joint
d5 = 0.100  # meters, distance between wrist joint and an imaginary point between fingers

# DH Parameter Table
dh_params = [  # theta_offset,  alpha    a,    d 
               [      0,         pi/2,   0,   d1 ], # joint 1, base rotation
               [   pi/2,            0,  a2,    0 ], # joint 2, shoulder bend
               [      0,            0,  a3,    0 ], # joint 3, elbow bend
               [   pi/2,         pi/2,   0,    0 ], # joint 4, wrist bend
               [      0,            0,   0,   d5 ], # joint 5, wrist rotation
            ]

cos_alpha = [ np.cos(alpha) for theta, alpha, a, d in dh_params ]
sin_alpha = [ np.sin(alpha) for theta, alpha, a, d in dh_params ]

def xform_matrix(i, theta_i):
    """
    Returns the 4x4 transform matrix for joint i.
    Parameters:
      i = integer from 1 to 5
      theta_i = current angle of joint i, in radians, where typically
                joint_min_rad[i-1] <= theta_i <= joint_max_rad[i-1]
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

def calculate_joint_angles_delta(q_current, target):
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
    q_delta = J_pinv @ (target - end_pos)
    q_delta = (q_delta / np.linalg.norm(q_delta)) * 0.2 * math.sqrt(err)
    
    return q_delta

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

