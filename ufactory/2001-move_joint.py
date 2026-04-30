#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Joint
Refactored to work from the ufactory/ directory alongside robot.conf
Run as: python3 2001-move_joint.py
"""

import os
import sys
import time
import math

from xarm.wrapper import XArmAPI


#######################################################
# IP resolution: command line arg > robot.conf > manual input
if len(sys.argv) >= 2:
    ip = sys.argv[1]
    print(f"From command line, xArm ip = {ip}")
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        # Look for robot.conf in the same directory as this script
        conf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.conf')
        parser.read(conf_path)
        ip = parser.get('xArm', 'ip')
        print(f"From robot.conf, xArm ip = {ip}")
    except:
        ip = input('Please input the xArm ip address: ')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

print("Going home...")
arm.move_gohome(wait=True)

# --- Move using degrees ---
print("\n--- Moving in degrees ---")
speed = 50  # deg/s

arm.set_servo_angle(angle=[90, 0, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[90, 0, -60, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[90, -30, -60, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[0, -30, -60, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[0, 0, -60, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], speed=speed, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))


# --- Move using radians ---
print("\n--- Moving in radians ---")
arm.move_gohome(wait=True)
speed = math.radians(50)  # rad/s

arm.set_servo_angle(angle=[math.radians(90), 0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[math.radians(90), 0, math.radians(-60), 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[math.radians(90), math.radians(-30), math.radians(-60), 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[0, math.radians(-30), math.radians(-60), 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[0, 0, math.radians(-60), 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))

arm.set_servo_angle(angle=[0, 0, 0, 0, 0, 0], speed=speed, is_radian=True, wait=True)
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))


print("\nGoing home and disconnecting...")
arm.move_gohome(wait=True)
arm.disconnect()
print("Done.")
