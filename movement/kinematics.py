import numpy as np

def map_normalized_to_radians(t, lower_bound, upper_bound):
    """
    Maps a normalized value t ∈ [0, 1] to a range of radians [lower_bound, upper_bound].
    """
    if upper_bound < lower_bound:
        upper_bound += 2 * np.pi  # Handle circular wraparound
    return lower_bound + t * (upper_bound - lower_bound)

def anthroarm_diff(q):
    """
    Differential Kinematics of an Anthropomorphic Arm with Spherical Wrist
    Calculates the Jacobian matrix J.

    Parameters:
    q : array-like
        A list or array containing joint angles [theta1, theta2, theta3, theta4, theta5, theta6].

    Returns:
    J : np.ndarray
        The Jacobian matrix relating joint velocities to end-effector velocities.
    """
    # Robot Morphological Parameters
    a2 = 0.155  # Length of link 2 (shoulder -> elbow)
    d4 = 0.096  # Length of link 4 (elbow -> wrist)
    d6 = 0.100  # Length of link 6 (wrist -> end)

    # Trigonometric shortcuts
    c1, c2, c3, c4, c5, c6 = np.cos(q[0]), np.cos(q[1]), np.cos(q[2]), np.cos(q[3]), np.cos(q[4]), np.cos(q[5])
    s1, s2, s3, s4, s5, s6 = np.sin(q[0]), np.sin(q[1]), np.sin(q[2]), np.sin(q[3]), np.sin(q[4]), np.sin(q[5])
    c23, s23 = np.cos(q[1] + q[2]), np.sin(q[1] + q[2])

    # Position of the end-effector
    p = np.array([
        a2 * c1 * c2 + d4 * c1 * s23 + d6 * (c1 * (c23 * c4 * s5 + s23 * c5) + s1 * s4 * s5),
        a2 * s1 * c2 + d4 * s1 * s23 + d6 * (s1 * (c23 * c4 * s5 + s23 * c5) - c1 * s4 * s5),
        a2 * s2 - d4 * c23 + d6 * (s23 * c4 * s5 - c23 * c5)
    ])

    # Homogeneous transformation matrices
    A01 = np.array([[c1, 0, s1, 0],
                    [s1, 0, -c1, 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
    A12 = np.array([[c2, -s2, 0, a2 * c2],
                    [s2, c2, 0, a2 * s2],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    A23 = np.array([[c3, 0, s3, 0],
                    [s3, 0, -c3, 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
    A34 = np.array([[c4, 0, -s4, 0],
                    [s4, 0, c4, 0],
                    [0, -1, 0, d4],
                    [0, 0, 0, 1]])
    A45 = np.array([[c5, 0, s5, 0],
                    [s5, 0, -c5, 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
    A56 = np.array([[c6, -s6, 0, 0],
                    [s6, c6, 0, 0],
                    [0, 0, 1, d6],
                    [0, 0, 0, 1]])

    # Composed transformations
    T02 = A01 @ A12
    T03 = T02 @ A23
    T04 = T03 @ A34
    T05 = T04 @ A45

    # Link-end positions
    p0 = np.array([0, 0, 0])
    p1 = p0
    p2 = T02[:3, 3]
    p3 = T03[:3, 3]
    p4 = T04[:3, 3]
    p5 = T05[:3, 3]

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
        np.column_stack((np.cross(z0, (p - p0)), np.cross(z1, (p - p1)), np.cross(z2, (p - p2)),
                         np.cross(z3, (p - p3)), np.cross(z4, (p - p4)), np.cross(z5, (p - p5)))),
        np.column_stack((z0, z1, z2, z3, z4, z5))
    ))

    return J

def anthroarm_dm(q):
    """
    Direct Kinematics of an Anthropomorphic Arm with Spherical Wrist
    Calculates matrix T and vectors p, a, s, n.

    Parameters:
    q : array-like
        A list or array containing joint angles [theta1, theta2, theta3, theta4, theta5, theta6].

    Returns:
    T : np.ndarray
        Homogeneous transformation matrix.
    p : np.ndarray
        Position of the end-effector in 3D space.
    a : np.ndarray
        Orientation vector aligned with the direction of the final link.
    s : np.ndarray
        Orientation vector orthogonal to the direction of the final link and aligned with the width of the grabber.
    n : np.ndarray
        Orientation vector orthogonal to both a and s.
    """
    # Robot Morphological Parameters
    a2 = 0.155  # Length of link 2 (shoulder -> elbow)
    d4 = 0.096  # Length of link 4 (elbow -> wrist)
    d6 = 0.100  # Length of link 6 (wrist -> end)

    # Trigonometric shortcuts
    c1, c2, c4, c5, c6 = np.cos(q[0]), np.cos(q[1]), np.cos(q[3]), np.cos(q[4]), np.cos(q[5])
    s1, s2, s4, s5, s6 = np.sin(q[0]), np.sin(q[1]), np.sin(q[3]), np.sin(q[4]), np.sin(q[5])
    c23, s23 = np.cos(q[1] + q[2]), np.sin(q[1] + q[2])

    # Position of the End-Effector in 3D Space, w.r.t. the base
    p = np.array([
        a2 * c1 * c2 + d4 * c1 * s23 + d6 * (c1 * (c23 * c4 * s5 + s23 * c5) + s1 * s4 * s5),
        a2 * s1 * c2 + d4 * s1 * s23 + d6 * (s1 * (c23 * c4 * s5 + s23 * c5) - c1 * s4 * s5),
        a2 * s2 - d4 * c23 + d6 * (s23 * c4 * s5 - c23 * c5)
    ])

    # Orientation vectors
    n = np.array([
        c1 * (c23 * (c4 * c5 * c6 - s4 * s6) - s23 * s5 * c6) + s1 * (s4 * c5 * c6 + c4 * s6),
        s1 * (c23 * (c4 * c5 * c6 - s4 * s6) - s23 * s5 * c6) - c1 * (s4 * c5 * c6 + c4 * s6),
        s23 * (c4 * c5 * c6 - s4 * s6) + c23 * s5 * c6
    ])

    s = np.array([
        c1 * (-c23 * (c4 * c5 * s6 + s4 * c6) + s23 * s5 * s6) + s1 * (-s4 * c5 * s6 + c4 * c6),
        s1 * (-c23 * (c4 * c5 * s6 + s4 * c6) + s23 * s5 * s6) - c1 * (-s4 * c5 * s6 + c4 * c6),
        -s23 * (c4 * c5 * s6 + s4 * c6) - c23 * s5 * s6
    ])

    a = np.array([
        c1 * (c23 * c4 * s5 + s23 * c5) + s1 * s4 * s5,
        s1 * (c23 * c4 * s5 + s23 * c5) - c1 * s4 * s5,
        s23 * c4 * s5 - c23 * c5
    ])

    # Homogeneous Transformation Matrix
    T = np.vstack((
        np.column_stack((n, s, a, p)),
        np.array([0, 0, 0, 1])
    ))

    return T, p, a, s, n

def calculate_joint_angles_delta(q_current, target):
    """
    Calculate the joint angle changes required for a desired end-effector change.
    
    Parameters:
    q_current : array-like
        Current joint angles [theta1, theta2, theta3, theta4, theta5, theta6].
    delta_p : array-like
        Desired change in the end-effector position and/or orientation.
        This should be a 6-element array: [dx, dy, dz, droll, dpitch, dyaw].
    
    Returns:
    delta_q : np.ndarray
        The change in joint angles to achieve the desired end-effector change.
    """
    # Compute the Jacobian matrix at the current joint angles
    J = anthroarm_diff(q_current)
    J[:,4] = np.transpose(np.array([0,0,0,0,0,0]))
    J = J[0:3,:]

    end_pos = anthroarm_dm(q_current)[1]
    print(end_pos)
    
    # Compute the pseudo-inverse of the Jacobian
    J_pinv = np.linalg.pinv(J)
    
    q_delta = J_pinv @ (target - end_pos)
    q_delta = (q_delta / np.linalg.norm(q_delta))
    
    return q_delta
