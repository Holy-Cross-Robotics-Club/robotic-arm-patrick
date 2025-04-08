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

def clicks_to_radians(joint_id, clicks):
    cmin = joint_min_clk[joint_id]
    cmax = joint_max_clk[joint_id]
    rmin = joint_min_rad[joint_id]
    rmax = joint_max_rad[joint_id]
    return rmin + (clicks - cmin) * (rmax - rmin) / (cmax - cmin)

def radians_to_clicks(joint_id, radians):
    cmin = joint_min_clk[joint_id]
    cmax = joint_max_clk[joint_id]
    rmin = joint_min_rad[joint_id]
    rmax = joint_max_rad[joint_id]
    return int(cmin + (radians - rmin) * (cmax - cmin) / (rmax - rmin))

# --- Forward Kinematics via translated MATLAB model ---
def anthroarm_params(q):
    a = np.zeros(6)
    d = np.zeros(6)
    a[1] = 0.4  # Link 2 (shoulder to elbow)
    d[3] = 0.3  # Link 4 (elbow to wrist)
    d[5] = 0.2  # Link 6 (wrist to end-effector)

    c = np.cos(q)
    s = np.sin(q)
    c23 = np.cos(q[1] + q[2])
    s23 = np.sin(q[1] + q[2])

    px = a[1]*c[0]*c[1] + d[3]*c[0]*s23 + d[5]*(c[0]*(c23*c[3]*s[4] + s23*c[4]) + s[0]*s[3]*s[4])
    py = a[1]*s[0]*c[1] + d[3]*s[0]*s23 + d[5]*(s[0]*(c23*c[3]*s[4] + s23*c[4]) - c[0]*s[3]*s[4])
    pz = a[1]*s[1] - d[3]*c23 + d[5]*(s23*c[3]*s[4] - c23*c[4])
    position = np.array([px, py, pz])

    class Trig:
        def __init__(self, c, s):
            self.c = c
            self.s = s

    return Trig(c, s), position

def forward_kinematics(q):
    C, p = anthroarm_params(q)
    c23 = np.cos(q[1] + q[2])
    s23 = np.sin(q[1] + q[2])

    n = np.array([
        C.c[0]*(c23*(C.c[3]*C.c[4]*C.c[5] - C.s[3]*C.s[5]) - s23*C.s[4]*C.c[5]) + C.s[0]*(C.s[3]*C.c[4]*C.c[5] + C.c[3]*C.s[5]),
        C.s[0]*(c23*(C.c[3]*C.c[4]*C.c[5] - C.s[3]*C.s[5]) - s23*C.s[4]*C.c[5]) - C.c[0]*(C.s[3]*C.c[4]*C.c[5] + C.c[3]*C.s[5]),
        s23*(C.c[3]*C.c[4]*C.c[5] - C.s[3]*C.s[5]) + c23*C.s[4]*C.c[5]
    ])

    s = np.array([
        C.c[0]*(-c23*(C.c[3]*C.c[4]*C.s[5] + C.s[3]*C.c[5]) + s23*C.s[4]*C.s[5]) + C.s[0]*(-C.s[3]*C.c[4]*C.s[5] + C.c[3]*C.c[5]),
        C.s[0]*(-c23*(C.c[3]*C.c[4]*C.s[5] + C.s[3]*C.c[5]) + s23*C.s[4]*C.s[5]) - C.c[0]*(-C.s[3]*C.c[4]*C.s[5] + C.c[3]*C.c[5]),
        -s23*(C.c[3]*C.c[4]*C.s[5] + C.s[3]*C.c[5]) - c23*C.s[4]*C.s[5]
    ])

    a = np.array([
        C.c[0]*(c23*C.c[3]*C.s[4] + s23*C.c[4]) + C.s[0]*C.s[3]*C.s[4],
        C.s[0]*(c23*C.c[3]*C.s[4] + s23*C.c[4]) - C.c[0]*C.s[3]*C.s[4],
        s23*C.c[3]*C.s[4] - c23*C.c[4]
    ])

    T = np.eye(4)
    T[:3, 0] = n
    T[:3, 1] = s
    T[:3, 2] = a
    T[:3, 3] = p
    return T, p, a, s, n
