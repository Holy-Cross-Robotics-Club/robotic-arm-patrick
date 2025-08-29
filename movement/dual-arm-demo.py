#!/usr/bin/env python3

# Top-level program to demo a dual-arm scenario.
#
# This uses the kinematics model to make high-level moves.
# There are 3 "cup" positions, A, B, C.
# It can pick up a cup from any position, or put it down in any position.

from controller import Controller
import numpy as np
import time as clock
from robot import goto, goto_rest
import sys

if __name__ == "__main__":

    devs = Controller.enumerate_arms()
    print("\nDetected %d arms:" % (len(devs)))
    for i in range(len(devs)):
        print("  %d  - Device path %s" % (i+1, str(devs[i])))
    if len(devs) != 2:
        print("Two USB arms are required for this program.")
        sys.exit(1)

    armL = Controller(use_arm=True, use_sim=False, arm_dev=devs[0])
    armR = Controller(use_arm=True, use_sim=False, arm_dev=devs[1])

    armL.connect()
    armR.connect()

    goto(armL, [0, 0, 0.5], None, None)
    goto(armR, [0, 0, 0.5], None, None)

    armL.disconnect()
    armR.disconnect()
