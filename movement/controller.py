# controller.py
#
# Note: This file is maybe mis-named? It does not implement code for motion
# planning, doesn't use the kinematics models or anything like that.
#
# This code is for interfacing to the robot arm control board and/or a simulated
# robot arm. It provides a Controller object, containing six Servo objects. The
# servos can be operated on individually, or the Controller can execute batched
# commands across a set of servos.
#
# The goal here is to provide a higher level API for interacting with the arm,
# hiding the details of the byte-stream USB protocol the robot actually
# requires.
#
# This code can interface to a real arm, or to a simulated arm (see simulation
# directory), or both simultaneously. If both are active, then the movement
# commands are sent to both, but query commands go to the real arm.

from connection import Connection, DeadEnd
from simulation import Simulation
from parameters import *
import time as clock
import sys

class Servo:
    """Acts as an interface to control and read from the servos.
    Works with connection.py and/or simulation.py to parse requests to HID-USB format.

    TO AVOID: too many coordinate conversions.
    The kinematics needs radians: floating point numbers within some range like 0 to pi.
    The robot requires clicks (or "hex"): integers within some range like 150 to 950.
    It's okay to print degrees to the console, but don't calculate with it.
    """
    def __init__(self, controller, jid, sid, name, default_time=200):
        self.position_range = [None, None]
        self.jid = jid
        self.sid = sid
        self.name = name
        self.default_time = default_time
        self.position_range = [joint_min_clk[jid], joint_max_clk[jid]]
        self.radian_range =  [joint_min_rad[jid], joint_max_rad[jid]]
        self.__is_moving = False
        self.__target_position = None
        self.controller = controller
        self.curr_set_pos = None
        self.last_pos = None
        self.stasis = 10
    def get_position_hex(self):
        pkt = [85, 85, 4, 21, 1, self.sid]
        count, sid, lo, hi = self.controller.write_then_read_in(pkt, 21, 4)
        if count != 1 or sid != self.sid:
            print(f"ERROR: expecting count 1, sid {self.sid}, but got count {count}, sid {sid}")
        unsigned_clicks = hi * 256 + lo
        #return unsigned_clicks
        signed_clicks = unsigned_clicks if unsigned_clicks <= 32767 else unsigned_clicks - 65536
        return signed_clicks
    def get_position_radians(self):
        return self.hex_to_radians(self.get_position_hex())
    def get_position(self):
        """ Returns position in all three units: clicks, radians, and degrees """
        clicks = self.get_position_hex()
        rad = self.hex_to_radians(clicks)
        deg = np.degrees(rad)
        return clicks, rad, deg
    def is_moving(self):
        cur_pos = self.get_position_hex() 
        if self.curr_set_pos is not None:
            diff = abs(cur_pos - self.curr_set_pos)
            # print(f"DEBUG: joint={self.jid} {self.name} current={self.get_position_hex()} target={self.curr_set_pos}")
            if diff <= 16:
                # print(f"joint {self.jid} reached target")
                return False
        if self.last_pos is None:
            self.last_pos = cur_pos
            self.stasis = 0
            # print(f"joint {self.jid} stasis = 0")
            return True
        else:
            diff = cur_pos - self.last_pos
            self.last_pos = cur_pos
            if diff == 0:
                self.stasis += 1
                if self.stasis > 10:
                    # print(f"joint {self.jid} stasis = {self.stasis} > 0")
                    return False
            else:
                # print(f"joint {self.jid} stasis = 0")
                self.stasis = 0
                return True
    def wait_until_stopped(self):
        while self.is_moving():
            clock.sleep(0.05) # 50 ms
    def set_position_hex(self, pos, time=None):
        self.last_pos = None
        self.stasis = 0
        # print(f"joint {self.jid} stasis = 0")
        self.curr_set_pos = pos
        if time is None:
            time = self.default_time
        time_ms = max(1, min(65535, int(time)))
        # print(f"DEBUG: joint={self.jid} sid={self.sid} {self.name} current={self.get_position_hex()} target={self.curr_set_pos} time={time_ms}")
        # IMPORTANT: Whenever we call get_position, then immediately try to write data to the arm,
        # the second command seems to fail. There needs to be a 10ms or larger delay. This
        # doesn't seem to be documented anywhere.
        clock.sleep(0.020) # 20ms delay, sending move command after reading position fails if done too quickly??
        # Convert signed hex values to unsigned 
        if pos < 0:
            pos = pos + 65536
        # Warn if out of range, and clamp into range. Shouldn't be needed, but just in case.
        if pos < 0:
            print(f"INTERNAL ERROR: pos < 0 should never happen, but pos = {pos}")
            pos = 0
        elif pos > 65535:
            print(f"INTERNAL ERROR: pos > 65535 should never happen, but pos = {pos}")
            pos = 65535
        self.controller.write_out([85, 85, 8, 3, 1, (time_ms&0xff), ((time_ms>>8)&0xff), self.sid, (pos&0xff), ((pos>>8)&0xff)])
    def set_position_radians(self, rad, time=None):
        hex = self.hex_from_radians(rad)
        self.set_position_hex(hex, time)
        return hex
    def hex_to_radians(self, hex):
        # Check and warn if hex is outside our known valid range, force it back into range.
        if hex < self.position_range[0]:
            print(f"ERROR: OUT OF RANGE: joint={self.jid} sid={self.sid} {self.name} current={hex} valid_range={self.position_range[0]}...{self.position_range[1]}")
            hex = self.position_range[0]
        elif self.position_range[1] < hex:
            print(f"ERROR: OUT OF RANGE: joint={self.jid} sid={self.sid} {self.name} current={hex} valid_range={self.position_range[0]}...{self.position_range[1]}")
            hex = self.position_range[1]
        rad_span = (self.radian_range[1] - self.radian_range[0])
        hex_span = (self.position_range[1] - self.position_range[0])
        rad = self.radian_range[0] + (hex - self.position_range[0]) * rad_span / hex_span
        return rad
    def hex_from_radians(self, rad):
        # Check and warn if rad is outside our known valid range, for it back into range.
        if rad < self.radian_range[0]:
            print(f"ERROR: OUT OF RANGE: joint={self.jid} sid={self.sid} {self.name} rad={rad} valid_range={self.radian_range[0]}...{self.radian_range[1]}")
            rad = self.radian_range[0]
        elif self.radian_range[1] < rad:
            print(f"ERROR: OUT OF RANGE: joint={self.jid} sid={self.sid} {self.name} rad={rad} valid_range={self.radian_range[0]}...{self.radian_range[1]}")
            rad = self.radian_range[1]
        rad_span = (self.radian_range[1] - self.radian_range[0])
        hex_span = (self.position_range[1] - self.position_range[0])
        hex = self.position_range[0] + int((rad - self.radian_range[0]) * hex_span / rad_span)
        return hex

class Controller:
    product_id = 0x5750  # USB product ID for HiWonder xArm
    vendor_id = 0x0483   # USB vendor ID for HiWonder xArm

    @staticmethod
    def enumerate_arms():
        return Connection.enumerate_devices(Controller.product_id, Controller.vendor_id)

    @staticmethod
    def parse_args_for_arm(argv, allow_both=True):
        use_sim = False
        use_arm = False
        i = 1
        while i < len(argv):
            arg = argv[i]
            if arg == "--sim":
                use_sim = True
                del argv[i]
            elif arg == "--arm":
                use_arm = True
                del argv[i]
            elif allow_both and arg == "--both":
                use_sim = True
                use_arm = True
                del argv[i]
            else:
                i += 1
        while not use_sim and not use_arm:
            print("\nChoose an option:")
            print("  sim  - Use the browser-based simulation")
            print("  arm  - Connect to the physical robot arm")
            if allow_both:
                print("  both - Connect to the physical robot arm, and browser-based simulation")
                choice = input("Enter your choice, or hit enter to use both: ").strip().lower()
            else:
                choice = input("Enter your choice, or hit enter to use arm: ").strip().lower()
            if choice == "sim":
                use_sim = True
                print("NOTE: in future, you can use '--sim' as a command-line argument to skip this menu.")
            elif choice == "arm" or (choice == "" and not allow_both):
                use_arm = True
                print("NOTE: in future, you can use '--arm' as a command-line argument to skip this menu.")
            elif allow_both and (choice in [ "", "both" ]):
                use_sim = True
                use_arm = True
                print("NOTE: in future, you can use '--both' as a command-line argument to skip this menu.")
            else:
                if allow_both:
                    print("Sorry, that's not an option. Type 'sim' or 'arm' or 'both'.")
                else:
                    print("Sorry, that's not an option. Type 'sim' or 'arm' or 'both'.")
        dev = None
        if use_arm:
            devs = Controller.enumerate_arms()
            if not devs:
                raise AssertionError("USB connection for arm not detected. Are you sure the xArm is plugged in and turned on?")
            if len(devs) == 1:
                dev = devs[0]
            else:
                while not dev:
                    print("\nDetected %d arms:" % (len(devs)))
                    for i in range(len(devs)):
                        print("  %d  - Device path %s" % (i+1, str(devs[i])))
                    choice = input("Pick one, or hit enter to use the first one listed: ").strip()
                    try:
                        choice = int(choice) if choice else 1
                    except:
                        choice = -1
                    if choice <= 0 or choice > len(devs):
                        print("Sorry, that's not an option. Enter a number between 1 and %d." % (len(devs)))
                        continue
                    dev = devs[choice-1]
            print("Connecting to USB device %s" % (str(dev)))
        arm = Controller(use_arm, use_sim, arm_dev=dev)
        return arm

    def __init__(self, use_arm=True, use_sim=False, arm_dev=None):
        if use_arm and use_sim:
            # If both the arm and the simulator are enabled, use the arm for the
            # primary connection, and the simulation in view-only mode.
            self.connection = Connection(arm_dev)
            self.vieweronly = Simulation()
        elif use_arm:
            # If using the arm but no simulator, disable the view-only mode.
            self.connection = Connection(arm_dev)
            self.vieweronly = DeadEnd()
        elif use_sim:
            # If using the simulator but no arm, the simulator is used as the
            # primary connection, and we disable the view-only mode.
            self.connection = Simulation()
            self.vieweronly = DeadEnd()
        else:
            raise Exception("Either use_arm or use_sim, or both, must be True.")
        self.gripper =  Servo(self, jid=5, sid=1, default_time=200, name="gripper")
        self.hand  =    Servo(self, jid=4, sid=2, default_time=400, name="hand")
        self.wrist  =   Servo(self, jid=3, sid=3, default_time=800, name="wrist")
        self.elbow =    Servo(self, jid=2, sid=4, default_time=800, name="elbow")
        self.shoulder = Servo(self, jid=1, sid=5, default_time=800, name="shoulder")
        self.base =     Servo(self, jid=0, sid=6, default_time=800, name="base")
        self.joints = [ self.base, self.shoulder, self.elbow, self.wrist, self.hand, self.gripper ]
    def connect(self):
        self.connection.connect()
        self.vieweronly.connect()
    def disconnect(self):
        self.connection.close()
        self.vieweronly.close()
    def home(self):
        for joint in self.joints:
            joint.set_position_radians(0, 5)
    def write_out(self, pkt):
        self.connection.write_out(pkt)
        self.vieweronly.write_out(pkt)
    def write_then_read_in(self, pkt, cmd, payload_len):
        self.connection.write_out(pkt)
        return self.connection.read_in(cmd, payload_len)
    def get_multiple_position_hex(self, joints):
        """ Returns multiple positions in clicks """
        if not joints:
            return [ ]
        n = len(joints)
        pkt = [85, 85, 3+n, 21, n] + [joint.sid for joint in joints]
        result = self.write_then_read_in(pkt, 21, 1 + 3*n)
        if not result:
            # retry? Sometimes one of the servos fails to respond, leading to a size mismatch
            # FROM BEN: this is an issue with faulty cables!! after replacing cables, this issue went away
            clock.sleep(0.020) # 20ms delay
            result = self.write_then_read_in(pkt, 21, 1 + 3*n)
            if not result:
                print("FATAL ERROR, two servo failures in a row")
                sys.exit(1)
        if result[0] != n:
            print(f"ERROR: expecting count {n}, but got count {result[0]}")
        pos = [ ]
        for i in range(len(joints)):
            sid, lo, hi = result[1+3*i:1+3*i+3]
            if sid != joints[i].sid:
                print(f"ERROR: expecting sid {joints[i].sid}, but got sid {sid}")
            unsigned_clicks = hi * 256 + lo
            signed_clicks = unsigned_clicks if unsigned_clicks <= 32767 else unsigned_clicks - 65536
            pos.append(signed_clicks)
        return pos
    def get_multiple_position_radians(self, joints):
        """ Returns multiple positions in radians """
        clicks = self.get_multiple_position_hex(joints)
        return [joint.hex_to_radians(clicks) for joint, clicks in zip(joints, clicks)]
    def get_multiple_position(self, joints):
        """ Returns multiple positions in all three units: clicks, radians, and degrees """
        clicks = self.get_multiple_position_hex(joints)
        rads = [joint.hex_to_radians(clicks) for joint, clicks in zip(joints, clicks)]
        degs = [np.degrees(rad) for rad in rads]
        return clicks, rads, degs
    def set_multiple_position_hex(self, joints, clicks, time=None):
        default_time = time
        adjusted_clicks = []
        if not joints:
            return
        n = len(joints)
        for joint, pos in zip(joints, clicks):
            joint.curr_set_pos = pos
            if default_time is None:
                default_time = joint.default_time
            else:
                default_time = min(default_time, joint.default_time)
            # Convert signed hex values to unsigned 
            if pos < 0:
                pos = pos + 65536
            # Warn if out of range, and clamp into range. Shouldn't be needed, but just in case.
            if pos < 0:
                print(f"INTERNAL ERROR: pos < 0 should never happen, but pos = {pos}")
                pos = 0
            elif pos > 65535:
                print(f"INTERNAL ERROR: pos > 65535 should never happen, but pos = {pos}")
                pos = 65535
            adjusted_clicks.append(pos)
        if time is None:
            time = default_time
        time_ms = max(1, min(65535, int(time)))
        # IMPORTANT: Whenever we call get_position, then immediately try to write data to the arm,
        # the second command seems to fail. There needs to be a 10ms or larger delay. This
        # doesn't seem to be documented anywhere.
        clock.sleep(0.020) # 20ms delay, sending move command after reading position fails if done too quickly??
        pkt = [ 85, 85, 5+3*n, 3, n, (time_ms&0xff), ((time_ms>>8)&0xff) ]
        for joint, pos in zip(joints, adjusted_clicks):
            pkt = pkt + [ joint.sid, (pos&0xff), ((pos>>8)&0xff) ]
        self.write_out(pkt)
    def set_multiple_position_radians(self, joints, rads, time=None):
        clicks = [joint.hex_from_radians(rad) for joint, rad in zip(joints, rads)]
        self.set_multiple_position_hex(joints, clicks, time)
        return clicks
    def q_current_radians(self):
        """ Returns a numpy array with radian angles of all joints """
        return np.array([joint.get_position_radians() for joint in self.joints])
    def q_current(self):
        """ Returns a 6x3 numpy array. The first row is click angles of all joints,
            second row is radian angles, third row is degree angles. """
        return np.array([joint.get_position() for joint in self.joints]).transpose()
    def qToDegreeString(self, q):
        """ Takes an nparray of radian angles, returns a nice string showing degrees """
        d0 = q[0] * 180/np.pi
        d1 = q[1] * 180/np.pi
        d2 = q[2] * 180/np.pi
        d3 = q[3] * 180/np.pi
        return "[ base = %4.0d°  shoulder = %4.0d°  elbow = %4.0d°  wrist = %4.0d° ]" % (d0, d1, d2, d3)
    def is_moving(self):
        return any([joint.is_moving() for joint in self.joints])
    def wait_until_stopped(self):
        while self.is_moving():
            clock.sleep(0.05) # 50 ms
    def qToString(self, q):
        """ Takes an nparray of radian angles, returns a nice string showing both hex and degrees """
        d0 = q[0] * 180/np.pi
        h0 = self.joints[0].hex_from_radians(q[0])
        d1 = q[1] * 180/np.pi
        h1 = self.joints[1].hex_from_radians(q[1])
        d2 = q[2] * 180/np.pi
        h2 = self.joints[2].hex_from_radians(q[2])
        d3 = q[3] * 180/np.pi
        h3 = self.joints[3].hex_from_radians(q[3])
        return "[ base = %4d = %4.0d°  shoulder = %4d = %4.0d°  elbow = %4d = %4.0d°  wrist = %4d = %4.0d° ]" % (
                h0, d0, h1, d1, h2, d2, h3, d3)

