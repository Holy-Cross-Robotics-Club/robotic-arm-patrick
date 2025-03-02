#!/usr/bin/env python3

# This uses the kinematics model to make high-level moves.
# There are 3 "cup" positions, A, B, C.
# It can pick up a cup from any position, or put it down in any position.

from controller import Controller
from connection import Connection
from simulation import Simulation
import numpy as np
from kinematics import *
import time as clock
from robot import goto, goto_rest
import sys

if __name__ == "__main__":

    use_sim = False
    use_arm = False

    cmd_idx = None
    for i, arg in enumerate(sys.argv[1:]):
        if arg == "--sim":
            use_sim = True
        elif arg == "--arm":
            use_arm = True
        elif arg == "--both":
            use_sim = True
            use_arm = True
        elif arg in ["--help", "-?"]:
            print("Usage:")
            print("  ./robot.py [options] point A        # point at cup position A (or B or C, or UP, or REST)")
            print("  ./robot.py [options] pickup A       # pick up the cup at position A (or B or C)")
            print("  ./robot.py [options] drop A         # drop cup at position A (or B or C)")
            print("Options:")
            print("  --sim          ... open the browser-based simulation")
            print("  --arm          ... connect to the physical robot arm")
            print("  --both         ... use both the simulation and physical arm")
            print("  --help, -?     ... show this message")
        elif arg.startswith("-"):
            print(f"Unrecognized option '{arg}'. Try '--help' instead.")
            sys.exit(1)
        else:
            cmd_idx = 1+i
            break

    args = sys.argv[cmd_idx:]

    if len(args) != 2:
        print("Missing arguments")
        print("Maybe try: ./robot.py --arm pickup A")
        print("Or try:    ./robot --help")
        sys.exit(1)

    action, cup = args
    cup = cup.upper()

    if action not in [ "point", "pickup", "drop" ]:
        print(f"Unknown action '{action}', should be point, pickup, or drop.")
        sys.exit(1)

    if cup == "A":
        x,y = 0.26, 0.20
    elif cup == "B":
        x,y = 0.26, 0.00
    elif cup == "C":
        x,y = 0.26, -0.20
    elif cup == "UP":
        x,y = 0.0, 0.0 # ignored
    elif cup == "REST":
        x,y = 0.0, 0.0 # ignored
    else:
        print(f"Unknown position '{cup}', should be A, B, or C.")
        sys.exit(1)

    while not use_sim and not use_arm:
        print("\nChoose an option:")
        print("  sim  - Use the browser-based simulation")
        print("  arm  - Connect to the physical robot arm")
        choice = input("Enter your choice, or hit enter to use both: ").strip().lower()
        if choice == "sim":
            use_sim = True
            print("NOTE: in future, you can use './main.py --sim' to skip this menu.")
        elif choice == "arm":
            use_arm = True
            print("NOTE: in future, you can use './main.py --arm' to skip this menu.")
        elif choice in [ "", "both" ]:
            use_sim = True
            use_arm = True
            print("NOTE: in future, you can use './main.py --both' to skip this menu.")
        else:
            print("Sorry, that's not an option. Type 'sim' or 'arm' or 'both'.")

    arm = Controller(use_arm, use_sim)
    arm.connect()

    if action == "point":
        if cup == "UP":
            goto(arm, [0, 0, 0.5], None, None)
        elif cup == "REST":
            goto_rest(arm)
        else:
            goto(arm, [x, y, 0.14], hand=0.0, grip=pi/2) # closed grip
    elif action == "pickup":
        goto(arm, [x, y, 0.12], attack=[-0.05, 0.05], hand=0.0, grip=0.0) # open grip, above table
        goto(arm, [x, y, 0.07], attack=0.0, hand=0.0, grip=0.0) # open grip, down
        goto(arm, [x, y, 0.07], attack=0.0, hand=0.0, grip=2.0) # partly closed grip, down
        clock.sleep(0.3)
        goto(arm, [x, y, 0.12], hand=0.0, grip=2.0) # partly closed grip, above table
        goto_rest(arm)
    elif action == "drop":
        goto(arm, [x, y, 0.12], attack=[-0.05, 0.05], hand=0.0, grip=2.0) # partly closed grip, above table
        goto(arm, [x, y, 0.07], attack=0.0, hand=0.0, grip=2.0) # partly closed grip, down
        clock.sleep(0.3)
        goto(arm, [x, y, 0.07], attack=0.0, hand=0.0, grip=0.0) # open grip, down
        goto(arm, [x, y, 0.12], hand=0.0, grip=0.0) # open grip, above table
        goto_rest(arm)

    arm.disconnect()
