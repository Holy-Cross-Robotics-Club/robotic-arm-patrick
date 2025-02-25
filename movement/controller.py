from connection import Connection, DeadEnd
from simulation import Simulation
from kinematics import *
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
    def get_position_hex(self):
        self.controller.connection.write_out([85, 85, 4, 21, 1, self.sid])
        self.controller.vieweronly.write_out([85, 85, 4, 21, 1, self.sid])
        result = self.controller.connection.read_in(21, 6)
        unsigned_clicks = (result[3] * 256 + result[2])
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
        if self.curr_set_pos is None:
            return False
        diff = abs(self.get_position_hex() - self.curr_set_pos)
        print(f"DEBUG: joint={self.jid} {self.name} current={self.get_position_hex()} target={self.curr_set_pos}")
        return False if diff <= 16 else True
    def set_position_hex(self, pos, time=None):
        self.curr_set_pos = pos
        if time is None:
            time = self.default_time
        time_ms = max(1, min(65535, int(time)))
        print(f"DEBUG: joint={self.jid} sid={self.sid} {self.name} current={self.get_position_hex()} target={self.curr_set_pos} time={time_ms}")
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
        self.controller.connection.write_out([85, 85, 8, 3, 1, (time_ms&0xff), ((time_ms>>8)&0xff), self.sid, (pos&0xff), ((pos>>8)&0xff)])
        self.controller.vieweronly.write_out([85, 85, 8, 3, 1, (time_ms&0xff), ((time_ms>>8)&0xff), self.sid, (pos&0xff), ((pos>>8)&0xff)])
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
    def __init__(self, use_arm=True, use_sim=False):
        if use_arm and use_sim:
            # If both the arm and the simulator are enabled, use the arm for the
            # primary connection, and the simulation in view-only mode.
            self.connection = Connection()
            self.vieweronly = Simulation()
        elif use_arm:
            # If using the arm but no simulator, disable the view-only mode.
            self.connection = Connection()
            self.vieweronly = DeadEnd()
        elif use_sim:
            # If using the simulator but no arm, the simulator is used as the
            # primary connection, and we disable the view-only mode.
            self.connection = Simulation()
            self.vieweronly = DeadEnd()
        else:
            raise Exception("Either use_arm or use_arm, or both, must be True.")
        self.gripper =  Servo(self, jid=5, sid=1, default_time=200, name="gripper")
        self.hand  =    Servo(self, jid=4, sid=2, default_time=400, name="hand")
        self.wrist  =   Servo(self, jid=3, sid=3, default_time=800, name="wrist")
        self.elbow =    Servo(self, jid=2, sid=4, default_time=800, name="elbow")
        self.shoulder = Servo(self, jid=1, sid=5, default_time=800, name="shoulder")
        self.base =     Servo(self, jid=0, sid=6, default_time=800, name="base")
        self.joints = [ self.base, self.shoulder, self.elbow, self.wrist, self.hand, self.gripper ]
        self.product_id = 0x5750
        self.vendor_id = 0x0483
    def connect(self):
        self.connection.connect(self.product_id, self.vendor_id)
        self.vieweronly.connect(self.product_id, self.vendor_id)
    def disconnect(self):
        self.connection.close()
        self.vieweronly.close()
    def home(self):
        for joint in self.joints:
            joint.set_position_radians(0, 5)
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

