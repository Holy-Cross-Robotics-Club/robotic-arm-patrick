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
    center = np.array([0.15, 0.0, 0.30]) # convert python list to numpy array
    radius = 0.06

    robot.goto(arm, np.array(home_coords), verbose=True, strategy=my_strategy)
    time.sleep(2)

    robot.goto(arm, center, verbose=True, strategy=my_strategy)
    time.sleep(2)

    for theta in np.linspace(0, 4*np.pi, 80):  # 2 spins
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        
        # Add bounce using sine wave
        z = center[2] + 0.05 * np.sin(3 * theta)
        time.sleep(0.05)

    robot.goto(arm, np.array(home_coords), verbose=True, strategy=my_strategy)

    arm.disconnect() # important: disconnect from the controller when done

if __name__ == "__main__":
    main()