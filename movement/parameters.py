
# This file stores the dh-parameter table, along with other physical parameters
# for the xArm robot arm. This file is meant to serve as a single definitive
# place to put the parameters, which can be included in other python programs.
# That way we avoid having multiple copies of the parameters, which will get out
# of sync as the parameters are adjusted or fine-tuned.
#
# The program in simulation/server.py uses these parameters as part of it's
# internal physics simulation of the arm. That program also passes these along
# to the browser-based visualization, which uses them to draw the arm as a 3D
# animation.
#
# The programs in movement/*.py all use these parameters as part of their
# kinematics code, etc.
#
# Note: This file appears twice, to keep python imports happy and simple:
#   movement/parameters.py    -- this actual file
#   simulation/parameters.py  -- a symlink (shortcut) to the actual file

import numpy as np

pi = np.pi

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

#                     base  shoulder     elbow     wrist      hand      grip
#  joint number =        0         1         2         3         4         5
#  servo number =        6         5         4         3         2         1
joint_min_clk = [        0,      144,       10,       70,      120,      155 ] # min extent, in clicks
joint_max_clk = [     1000,      880,      990,      930,      880,      666 ] # max extent, in clicks
joint_min_rad = [   -2.146,    -pi/2,   -2.079,   -1.806,    -pi/2,        0 ] # min extent, in radians
joint_max_rad = [     1.98,     pi/2,    2.079,    1.806,     pi/2,       pi ] # max extent, in radians

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

d1 = 0.080  # meters, height of shoulder joint above table or baseplate
a2 = 0.100  # meters, distance between shoulder joint and elbow joint
a3 = 0.095  # meters, distance between elbow joint and wrist joint
d5 = 0.160  # meters, distance between wrist joint and an imaginary point between fingers

# DH Parameter Table
dh_params = [  # theta_offset,  alpha    a,    d 
               [      0,         pi/2,   0,   d1 ], # joint 1, base rotation
               [   pi/2,           pi,  a2,    0 ], # joint 2, shoulder bend
               [      0,           pi,  a3,    0 ], # joint 3, elbow bend   # note... corrected by 180 deg rotation
               [   pi/2,         pi/2,   0,    0 ], # joint 4, wrist bend
               [      0,            0,   0,   d5 ], # joint 5, wrist rotation
            ]

