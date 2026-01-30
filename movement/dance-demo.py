# This is a demo showing how to move the robot via script
# Usage: python dance-demo.py

from controller import Controller
import sys
import robot
import numpy as np
import time

home_coords = [0.0, 0.0, 0.5] # saving home coordinates for convenience

def main():
    arm = Controller.parse_args_for_arm(sys.argv) # creating the Controller object
    arm.connect() # establishing the connection bewtween the Controller and the arm (or sim) itself

    # "shortcut": solves IK and sets joint angles directly
    # "standard": iteratively approximates direction, executes small movement, then revaluates
    my_strategy = "shortcut"

    # the destination (or "target") could be a list of Cartesian (x,y,z) coordinates
    target = [0.1, 0.1, 0.2]
    target = np.array(target) # convert python list to numpy array

    target2 = [0.17, 0.1, 0.2]
    target2 = np.array(target2) # convert python list to numpy array

    target3 = [0.17, 0.17, 0.2]
    target3 = np.array(target3) # convert python list to numpy array

    target4 = [0.1, 0.1, 0.2]
    target4 = np.array(target4) # convert python list to numpy array

    robot.goto(arm, target, verbose=True, strategy=my_strategy)
    time.sleep(2)
    robot.goto(arm, target2, verbose=True, strategy=my_strategy)
    time.sleep(2)
    robot.goto(arm, target3, verbose=True, strategy=my_strategy)
    time.sleep(2)
    robot.goto(arm, target4, verbose=True, strategy=my_strategy)

    arm.disconnect() # important: disconnect from the controller when done

if __name__ == "__main__":
    main()