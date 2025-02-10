#!/usr/bin/env python3

# This does not move the servos, it only monitors their positions and uses the
# kinematics model to compute the x,y,z coordinates of the end-effector.

from controller import Controller
from connection import Connection
from simulation import Simulation
from kinematics import *
import time as clock
import sys

if __name__ == "__main__":

    use_sim = False
    use_arm = False

    active = [ False ] * 6

    for arg in sys.argv[1:]:
        if arg == "--sim":
            use_sim = True
        elif arg == "--arm":
            use_arm = True
        elif arg in [ "--joint=0", "--servo=6", "--base"]:
            active[0] = True
        elif arg in [ "--joint=1", "--servo=5", "--shoulder" ]:
            active[1] = True
        elif arg in [ "--joint=2", "--servo=4", "--elbow" ]:
            active[2] = True
        elif arg in [ "--joint=3", "--servo=3", "--wrist" ]:
            active[3] = True
        elif arg in [ "--joint=4", "--servo=2", "--hand" ]:
            active[4] = True
        elif arg in [ "--joint=5", "--servo=1", "--gripper" ]:
            active[5] = True
        elif arg == "--all":
            active = [ True * 6 ]
        elif arg in ["--help", "-?"]:
            print("Usage: ./monitor.py [options]")
            print("Options:")
            print("  --sim          ... monitor the browser-based simulation")
            print("  --arm          ... connect to and monitor the physical robot arm")
            print("  --all          ... monitor all joints")
            print("  --joint=N      ... monitor joint N, numbered 0 to 5 from base to gripper")
            print("  --servo=S      ... monitor servo S, numbered 6 to 1 from base to gripper")
            print("  --base         ... monitor joint 0, servo 6, the base")
            print("  --shoulder     ... monitor joint 1, servo 5, the shoulder")
            print("  --elbow        ... monitor joint 2, servo 4, the elbow")
            print("  --wrist        ... monitor joint 3, servo 3, the wrist")
            print("  --hand         ... monitor joint 4, servo 2, the hand")
            print("  --gripper      ... monitor joint 5, servo 1, the gripper")
            print("  --help, -?     ... show this message")
        elif arg.startswith("-"):
            print(f"Unrecognized option '{arg}'. Try '--help' instead.")
            sys.exit(1)

    if not any(active):
        active = [ True ] * 6
        print("all active")

    while not use_sim and not use_arm:
        print("\nChoose an option:")
        print("  sim  - Monitor the browser-based simulation")
        print("  arm  - Connect to and monitor the physical robot arm")
        choice = input("Enter your choice, or hit enter to use both: ").strip().lower()
        if choice == "sim":
            use_sim = True
            print("NOTE: in future, you can use './main.py --sim' to skip this menu.")
        elif choice == "arm":
            use_arm = True
            print("NOTE: in future, you can use './main.py --arm' to skip this menu.")
        else:
            print("Sorry, that's not an option. Type 'sim' or 'arm'.")

    arm = Controller(use_arm, use_sim)
    arm.connect()

    print(active)
    if all(active):
        print_header = 0
        while True:
            if print_header <= 0:
                print_header = 30
                print("    base  shoulder     elbow     wrist  ==>         x          y          z")
                print("--------  --------  --------  --------      ---------  ---------  ---------")
            else:
                print_header -= 1
            q_current = arm.q_current_radians()
            end_pos = calculate_end_pos(q_current)
            print("%7.2f°  %7.2f°  %7.2f°  %7.2f°      %6.1f mm  %6.1f mm  %6.1f mm" % (
                q_current[0], q_current[1], q_current[2], q_current[3], 
                end_pos[0]*1000, end_pos[1]*1000, end_pos[2]*1000))
            clock.sleep(0.1)
    else:
        joint_indices = [idx for idx, isactive in enumerate(active) if isactive]
        joints = [arm.joints[idx] for idx in joint_indices]
        print_header = 0
        while True:
            if print_header <= 0:
                print_header = 30
                for joint in joints:
                    print("%12s" % (joint.name), end="  ")
                print()
                for joint in joints:
                    print("%12s" % ("-" * 12), end="  ")
                print()
            else:
                print_header -= 1
            for joint in joints:
                ticks = joint.get_position_hex()
                rad = joint.hex_to_radians(ticks)
                deg = rad * 180/np.pi
                print("%4d %6.2f°" % (ticks, deg), end="  ")
            print()
            clock.sleep(0.1)

