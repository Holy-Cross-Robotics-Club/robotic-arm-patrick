#!/usr/bin/env python3

# This program is designed to ensure that the user's arm is calibrated in the same manner as our code envisions. 
# The code will have each joint move 30 degrees before returning to neutral (0 degrees) 

import ctypes
ctypes.CDLL("/opt/homebrew/lib/libhidapi.0.14.0.dylib")
import hid

from controller import Controller
import numpy as np
import time
import sys

def demo_joint_movement(arm, angle_deg=25, delay=0.5):
    angle_rad = np.radians(angle_deg)

    print("\nStarting fixed joint demo...")

    # Combine all 6: 4 main joints + hand + gripper
    all_joints = arm.joints + [arm.hand, arm.gripper]

    for joint in all_joints:
        print(f"\nMoving '{joint.name}' to +{angle_deg}°")
        joint.set_position_radians(angle_rad)
        while arm.is_moving():
            time.sleep(0.05)
        input("Press Enter to return to 0...")

        print(f"Returning '{joint.name}' to 0°")
        joint.set_position_radians(0)
        while arm.is_moving():
            time.sleep(0.05)
        time.sleep(delay)

    print("\nDemo complete.")

if __name__ == "__main__":
    use_sim = "--sim" in sys.argv or "--both" in sys.argv
    use_arm = "--arm" in sys.argv or "--both" in sys.argv

    if not use_sim and not use_arm:
        print("Usage: ./joint_demo.py [--sim|--arm|--both]")
        sys.exit(1)

    arm = Controller(use_arm, use_sim)
    arm.connect()

    demo_joint_movement(arm)

    arm.disconnect()
