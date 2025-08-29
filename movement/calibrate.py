#!/usr/bin/env python3

# A top-level program for helping calibrate the robot arm.
#
# This moves only one joint at a time, allowing you to repeatedly specify an
# angle (in degrees) to test the calibration of that joint.

from controller import Controller
import time as clock
import numpy as np
import sys

if __name__ == "__main__":
    
    arm = Controller.parse_args_for_arm(sys.argv)

    joint_arg = None
    servo_time = None # for speed, use default

    for arg in sys.argv[1:]:
        if arg.startswith("--time="):
            servo_time = int(arg[7:])
        elif arg in [ "--joint=0", "--servo=6", "--base"]:
            joint_arg = "base"
        elif arg in [ "--joint=1", "--servo=5", "--shoulder" ]:
            joint_arg = "shoulder"
        elif arg in [ "--joint=2", "--servo=4", "--elbow" ]:
            joint_arg = "elbow"
        elif arg in [ "--joint=3", "--servo=3", "--wrist" ]:
            joint_arg = "wrist"
        elif arg in [ "--joint=4", "--servo=2", "--hand" ]:
            joint_arg = "hand"
        elif arg in [ "--joint=5", "--servo=1", "--gripper" ]:
            joint_arg = "gripper"
        elif arg in ["--help", "-?"]:
            print("Usage: ./calibrate.py [options]")
            print("Options:")
            print("  --sim          ... open the browser-based simulation")
            print("  --arm          ... connect to the physical robot arm")
            print("  --both         ... use both the simulation and physical arm")
            print("  --joint=N      ... control joint N, numbered 0 to 5 from base to gripper")
            print("  --servo=S      ... control servo S, numbered 6 to 1 from base to gripper")
            print("  --base         ... control joint 0, servo 6, the base")
            print("  --shoulder     ... control joint 1, servo 5, the shoulder")
            print("  --elbow        ... control joint 2, servo 4, the elbow")
            print("  --wrist        ... control joint 3, servo 3, the wrist")
            print("  --hand         ... control joint 4, servo 2, the hand")
            print("  --gripper      ... control joint 5, servo 1, the gripper")
            print("  --help, -?     ... show this message")
        elif arg.startswith("-"):
            print(f"Unrecognized option '{arg}'. Try '--help' instead.")
            sys.exit(1)

    arm.connect()

    joint = None
    while joint is None:
        if joint_arg is not None:
            choice = joint_arg
        else:
            print("\nChoose a joint:")
            print("  joint 0 / servo 6 / base")
            print("  joint 1 / servo 5 / shoulder")
            print("  joint 2 / servo 4 / elbow")
            print("  joint 3 / servo 3 / wrist")
            print("  joint 4 / servo 2 / hand")
            print("  joint 5 / servo 1 / gripper")
            choice = input("Enter your choice: ").strip().lower()
        if choice in [ "0", "j0", "j 0", "joint 0", "s0", "s 0", "servo 6", "b", "base"]:
            joint = arm.base
        elif choice in [ "1", "j1", "j 1", "joint 1", "s5", "s 5","servo 5", "s", "shoulder" ]:
            joint = arm.shoulder
        elif choice in [ "2", "j2", "j 2", "joint 2", "s4", "s 4","servo 4", "e", "elbow" ]:
            joint = arm.elbow
        elif choice in [ "3", "j3", "j 3", "joint 3", "s3", "s 3","servo 3", "w", "wrist" ]:
            joint = arm.wrist
        elif choice in [ "4", "j4", "j 4", "joint 4", "s2", "s 2","servo 2", "h", "hand" ]:
            joint = arm.hand
        elif choice in [ "5", "j5", "j 5", "joint 5", "s1", "s 1","servo 1", "g", "gripper" ]:
            joint = arm.gripper
        else:
            print("Sorry, that's not a choice. Try 'base' or 'servo 5' for example.")
            joint_arg = None
            continue
        if joint_arg is None:
            print(f"NOTE: in future, you can use option '--joint={joint.jid}' or '--servo={joint.sid}' or '--{joint.name}'")

    delay = servo_time if servo_time is not None else joint.default_time
    while True:
        clicks, rad, deg = joint.get_position()
        print(f"Current {joint.name} (joint {joint.jid}, servo {joint.sid}) is at {clicks} clicks, {deg} degrees")
        while joint.is_moving():
            #q_current = np.array([joint.get_position_radians() for joint in arm.joints])
            #end_pos = calculate_end_pos(q_current)
            #print(f"  servos = {arm.qToString(q_current)} so end_pos = {cartesianToString(end_pos)}")
            clock.sleep(delay/1000)
            continue
        s = input(f"Type an angle for {joint.name} in degrees: ")
        if not s:
            continue
        deg = float(s)
        rad = np.radians(deg)
        print(f"Setting {joint.name} (joint {joint.jid}, servo {joint.sid}) radians to {rad}, over approx {delay} ms")
        joint.set_position_radians(rad, servo_time)
    
