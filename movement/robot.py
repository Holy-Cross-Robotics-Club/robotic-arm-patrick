#!/usr/bin/env python3

# This is a top-level program that uses the kinematics model to move the arm,
# given x,y,z coordinates. It can also be included in other programs as a
# library of path-planning function.
# 
# Note: This file is maybe misleadingly named? Maybe this is better thought of
# as the "controller" half of a "controller+kinematics" motion planning approach.
#
# goto() is the primary function here. Given a target x,y,z point in space, and
# optionally some other parameters. It then:
#  - calls out to the kinematics model to figure out the joint angles needed.
#  - calls out to controller.py to move the actual arm.
#  - uses a control/feedback loop, if needed, to ensure it reaches the target.
#
# Two different motion-planning approaches are implemented:
#
#  - "shortcut" motion planning, using shortcut_kinematics.py, this amounts to:
#    1. Calculate the final motor positions corresponding to the target.
#    2. Turn *all* the motors on, sending them directly to those positions.
#    3. Wait for motors to arrive. That's it. No iteration here.
#
#  - "iterative" motion planning, using standard_kinematics.py, this amounts to:
#    1. Calculate the joint-space vector pointing (in cartesian space) towards the
#       target.
#    2. Move motors in that direction a small step.
#    3. Repeat, iteratively, using feedback to monitor progress.
#    4. If the arm stalls, then move back to some home/rest position and try
#       again. This might happen if the model is trying to exceed a joint limit,
#       or if we get trapped in a singularity.
#    5. If we stall out too many times, give up.
#
# Other motion-planning approaches are possible and have been discussed.

from controller import Controller
from connection import Connection
from simulation import Simulation
from direct_kinematics import *
from shortcut_kinematics import shortcut_solve
from standard_kinematics import calculate_joint_angles_delta
import time as clock
import sys


next_print_header = 0
def print_pos(q_current, end_pos, err, q_new=None):
    global next_print_header
    if next_print_header <= 0:
        next_print_header = 30
        print("    base  shoulder     elbow     wrist  ==>         x          y          z        err")
        print("--------  --------  --------  --------      ---------  ---------  ---------  ---------")
    else:
        next_print_header -= 1
    if err > 1.0:
        errmsg = "inf mm"
    else:
        errmsg = "%6.1f mm" % (err*1000.0)
    print("%7.2f°  %7.2f°  %7.2f°  %7.2f°      %6.1f mm  %6.1f mm  %6.1f mm  %9s" % (
        q_current[0]*180/np.pi, q_current[1]*180/np.pi,
        q_current[2]*180/np.pi, q_current[3]*180/np.pi, 
        end_pos[0]*1000, end_pos[1]*1000, end_pos[2]*1000, errmsg))
    if q_new is not None:
        print("%7.2f°  %7.2f°  %7.2f°  %7.2f°      (target angles)" % (
            q_new[0]*180/np.pi, q_new[1]*180/np.pi,
            q_new[2]*180/np.pi, q_new[3]*180/np.pi))

def goto_rest(arm):
    arm.base.set_position_radians(0)
    arm.shoulder.set_position_radians(np.radians(10))
    arm.elbow.set_position_radians(np.radians(60))
    arm.wrist.set_position_radians(np.radians(-40))
    arm.hand.set_position_radians(0)
    while arm.is_moving():
        clock.sleep(0.1)

def goto(arm, cart_target, attack=None, hand=None, grip=None, strategy="shortcut", verbose=False):
    nearby_target, moved = nearest_reachable_point(cart_target)
    if moved:
        print(f"NOTE: {cartesianToString(cart_target)} is outside the reach of the arm.")
        print(f"NOTE: {cartesianToString(nearby_target)} will be targetted instead.")
        next_print_header = 0
    if strategy == "shortcut":
        shortcut_goto(arm, nearby_target, attack, hand, grip, verbose=verbose)
    elif strategy == "standard":
        iterative_goto(arm, nearby_target, hand, grip, verbose)
    else:
        print(f"ERR: bad strategy {strategy}")
        # TODO: other strategies are possible, like:
        #  - a "modified-standard" (or "planar-standard"?) strategy, that uses a
        #    shortcut approach for the base rotation, and a standard
        #    jacobian-style model in 2D for all the other servos.
        #  - an "iterative-shortcut" strategy, by plotting a sequence of points
        #    in cartesian space, then using the shortcut_kinematics to traverse
        #    each point.
        #  - a "biased-standard" strategy, using standard_kinematics but biasing
        #    the results towards the center-position of each motor, or otherwise
        #    adjusting the results to avoid singularties and joint limits.
        #  - a "planning" strategy, where we evaluate possible paths
        #    through cartesian space, pick a favorite one, then use one of the
        #    kinematics models to follow the path.

def shortcut_goto(arm, cart_target, attack=None, hand=None, grip=None, verbose=False):

    q_current = np.array(arm.get_multiple_position_radians(arm.joints))
    end_pos = calculate_end_pos(q_current)
    err = np.linalg.norm(cart_target - end_pos)

    q_new = shortcut_solve(q_current, cart_target, attack=attack, verbose=verbose)
    if not q_new:
        print("Target is not within reach. Sorry.")
        return

    if hand is not None:
        arm.hand.set_position_radians(hand)
    if grip is not None:
        arm.gripper.set_position_radians(grip)

    if verbose:
        print_pos(q_current, end_pos, err, q_new)

    #ms_per_radian = 500 # about 1 second per 30 degrees
    ms_per_radian = 750 # about 1.5 second per 30 degrees
    #ms_per_radian = 1000 # about 2 second per 30 degrees
    #ms_per_radian = 1500 # about 3 second per 30 degrees
    rad = max([abs(q_current[i] - q_new[i]) for i in range(4)])
    servo_time = rad * ms_per_radian
    if verbose:
        print(f"Moving {np.degrees(rad)} degrees over {servo_time/1000} seconds.")
    arm.set_multiple_position_radians(arm.joints[0:4], q_new[0:4], servo_time)

    arm.wait_until_stopped()
    print("STILL")

def iterative_goto(arm, cart_target, attack=None, hand=None, grip=None, verbose=False):
    # THREE parameters control the speed of the arm movements
    # step_size: (radians) used by the kinematics model as the maximum
    #            amount to move the motors in each step along the gradient.
    #            Higher means bigger steps.
    # step_time: (in seconds) used below, how often to re-calculate
    #            and adjust the trajectory.
    #            Lower means quicker adjustements.
    # servo_time: (in milliseconds) used by servos, controls how
    #            quickly servo attemps to move to each new position.
    #            Lower means faster movement.
    #step_size = 0.0875 # about 5 degrees
    step_size = 0.175 # about 10 degrees
    #step_size = 0.785 # about 45 degrees
    step_time = 0.1 # seconds
    servo_time = 50 # milliseconds

    # how close must we be before declaring victory?
    # precision = 0.001 # aiming for err < 1mm
    precision = 0.008 # aiming for err < 8mm

    # start moving
    preverr = 100.0
    stall = 0
    attempted_reset = False

    if hand is not None:
        arm.hand.set_position_radians(hand)
    if grip is not None:
        arm.gripper.set_position_radians(grip)

    # TODO: if angle-of-attack is not None, include it in kinematics

    while True:
        clock.sleep(step_time)
        q_current = np.array(arm.get_multiple_position_radians(arm.joints))
        end_pos = calculate_end_pos(q_current)
        err = np.linalg.norm(cart_target - end_pos)
        #if verbose:
        #    print_pos(q_current, end_pos, err)
        if err < precision:
            if verbose:
                print(f"reached target position, err = {err*1000} mm")
            break
        elif err < preverr:
            stall = 0
        elif stall > 20:
            if attempted_reset:
                print("no progress after a reset and 20 more tries, giving up")
                break
            else:
                print("no progress after 20 tries, resetting to home and trying again")
                arm.home()
                clock.sleep(2)
                preverr = 100
                next_print_header = 0
                continue
        else:
            print(f"progress stalled (attemped {stall} of 20) with err {err*1000} mm...")
            stall += 1
        preverr = err
        q_delta, err = calculate_joint_angles_delta(q_current, cart_target, step_size, verbose=True)
        q_new = np.array(q_current) + q_delta
        if verbose:
            print_pos(q_current, end_pos, err, q_new)
        arm.set_multiple_position_radians(arm.joints[0:4], q_new[0:4], servo_time)

    if hand is not None:
        arm.hand.wait_until_stopped()
    if grip is not None:
        arm.gripper.wait_until_stopped()


if __name__ == "__main__":

    use_sim = False
    use_arm = False
    strategy = None

    cmd_idx = None
    for i, arg in enumerate(sys.argv[1:]):
        if arg == "--sim":
            use_sim = True
        elif arg == "--arm":
            use_arm = True
        elif arg == "--both":
            use_sim = True
            use_arm = True
        elif arg.startswith("--strategy="):
            _, _, strategy = arg.partition("=")
        elif arg in ["--help", "-?"]:
            print("Usage:")
            print("  ./robot.py [options] home                 # go to x=0.0 y=0.0 z=0.5, in meters")
            print("  ./robot.py [options] goto x y z [attack]  # go to coordinates, in meters, w/w/o attack angle, in degrees")
            print("  ./robot.py [options] move dx dy dz  # move a distance, in meters")
            print("Note: underscore '_' can be used to omit a coordinate. For example:")
            print("   ./robot --sim goto _ _ 0.5    # in simulation, leave x, y alone, go to z=0.5")
            print("   ./robot --arm move 0.2 _ _    # with real arm, move x by +0.2 meters")
            print("Options:")
            print("  --sim          ... open the browser-based simulation")
            print("  --arm          ... connect to the physical robot arm")
            print("  --both         ... use both the simulation and physical arm")
            print("  --strategy=X   ... use strategy X for motion planning")
            print("  --arm          ... connect to the physical robot arm")
            print("  --help, -?     ... show this message")
        elif arg.startswith("-"):
            print(f"Unrecognized option '{arg}'. Try '--help' instead.")
            sys.exit(1)
        else:
            cmd_idx = 1+i
            break

    if cmd_idx is None:
        action = [ "favorite" ]
    else:
        action = sys.argv[cmd_idx:]

    while not use_sim and not use_arm:
        print("\nChoose an option:")
        print("  sim  - Use the browser-based simulation")
        print("  arm  - Connect to the physical robot arm")
        choice = input("Enter your choice, or hit enter to use both: ").strip().lower()
        if choice == "sim":
            use_sim = True
            print("NOTE: in future, you can use './robot.py --sim' to skip this menu.")
        elif choice == "arm":
            use_arm = True
            print("NOTE: in future, you can use './robot.py --arm' to skip this menu.")
        elif choice in [ "", "both" ]:
            use_sim = True
            use_arm = True
            print("NOTE: in future, you can use './robot.py --both' to skip this menu.")
        else:
            print("Sorry, that's not an option. Type 'sim' or 'arm' or 'both'.")

    if strategy is None:
        print("\nChoose a motion-planning strategy:")
        print("  standard  - Use an iterative Jacobian planner")
        print("  shortcut  - Use the single-step shortcut planner")
        strategy = input("Enter your choice, or hit enter to use shortcut: ").strip().lower()
        if strategy == "":
            strategy = "shortcut"
        print(f"NOTE: in future, you can use './robot.py --strategy={strategy}' to skip this menu.")

    arm = Controller(use_arm, use_sim)
    arm.connect()

    q_current = np.array(arm.get_multiple_position_radians(arm.joints))
    end_pos = calculate_end_pos(q_current)

    print_pos(q_current, end_pos, 100.0)

    a = None
    if len(action) == 0 or action[0] == "favorite":
        dest_coords = [0.1, 0.1, 0.2] # an arbitrary favorite position
    elif len(action) == 1 and action[0] == "home":
        dest_coords = [0.0, 0.0, 0.5]
    elif len(action) == 4 or len(action) == 5 and action[0] == "goto":
        x = end_pos[0] if action[1] == '_' else float(action[1])
        y = end_pos[1] if action[2] == '_' else float(action[2])
        z = end_pos[2] if action[3] == '_' else float(action[3])
        a = np.radians(float(action[4])) if len(action) == 5 else None
        dest_coords = [x, y, z]
    elif len(action) == 4 and action[0] == "move":
        dx = 0.0 if action[1] == '_' else float(action[1])
        dy = 0.0 if action[2] == '_' else float(action[2])
        dz = 0.0 if action[3] == '_' else float(action[3])
        dest_coords = [end_pos[0]+dx, end_pos[1]+dy, end_pos[2]+dz]
    else:
        print("Bad arguments. Try these examples:")
        print("  goto 0.1 0.1 0.25   # go to the specified x,y,z coordinates, in meters")
        print("  goto _ _ 0.25       # go to the specified z coordinate, in meters, leave x and y alone")
        print("  move 0.1 _ -0.05    # move by the specfied dx and dz, in meters, leave y alone")
        print("  home                # move to home position at x=0.0 y=0.0 z=0.5")
        sys.exit(0)

    # adjust target so it is within reach
    cart_target = np.transpose(np.array(dest_coords))
  
    goto(arm, cart_target, attack=a, verbose=False, strategy=strategy)

    arm.disconnect()
