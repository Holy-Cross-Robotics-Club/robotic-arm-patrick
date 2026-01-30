# This is a demo showing how to move the robot via script
# Usage: python dance-demo.py

from controller import Controller
import sys
import robot
import numpy as np

home_coords = [0.0, 0.0, 0.5] # saving home coordinates for convenience

def main():
    arm = Controller.parse_args_for_arm(sys.argv) # creating the Controller object
    arm.connect() # establishing the connection bewtween the Controller and the arm (or sim) itself

    # "shortcut": solves IK and sets joint angles directly
    # "standard": iteratively approximates direction, executes small movement, then revaluates
    strategy = "shortcut"

    # the destination (or "target") could be a list of Cartesian (x,y,z) coordinates
    target = [0.1, 0.1, 0.2]
    target = np.array(target) # convert python list to numpy array

    robot.goto(arm, target, verbose=True, strategy=strategy)

    arm.disconnect() # important: disconnect from the controller when done

if __name__ == "__main__":
    main()