from connection import Connection
from simulation import Simulation
from kinematics import *
import time as clock
import sys

class Servo:
    """Acts as an interface to control and read from the servos.
    Works with connection.py to parse requests to HID-USB format.

    TO AVOID: too many coordinate conversions.
    The kinematics needs radians: floating point numbers within some range like 0 to pi.
    The robot requires clicks (or "hex"): integers within some range like 150 to 950.
    It's okay to print degrees to the console, but don't calculate with it.
    """
    def __init__(self, controller, jid, sid):
        self.position_range = [None, None]
        self.jid = jid
        self.sid = sid
        self.position_range = [joint_min_clk[jid-1], joint_max_clk[jid-1]]
        self.radian_range =  [joint_min_rad[jid-1], joint_max_rad[jid-1]]
        self.__is_moving = False
        self.__target_position = None
        self.controller = controller
        self.curr_set_pos = None
    def get_position_hex(self):
        self.controller.connection.write_out([85, 85, 4, 21, 1, self.sid])
        result = self.controller.connection.read_in(21, 6)
        hex = (result[3] * 256 + result[2]) # returns hex
        return hex
    def get_position_radians(self):
        return self.hex_to_radians(self.get_position_hex())
    def is_moving(self):
        if self.curr_set_pos is None:
            return False
        diff = abs(self.get_position_hex() - self.curr_set_pos)
        print(f"{self.get_position()} (GET) and {self.curr_set_pos} (SET)")
        return False if diff <= 16 else True
    def set_position_hex(self, pos, time):
        #clock.sleep(0.03)
        self.curr_set_pos = pos
        # print(f"Servo {self.sid}: Set position is now {self.curr_set_pos}")
        self.controller.connection.write_out([85, 85, 8, 3, 1, 0, time, self.sid, (pos&0xff), ((pos>>8)&0xff)])
    def set_position_radians(self, rad, time):
        hex = self.hex_from_radians(rad)
        self.set_position_hex(hex, time)
        return hex
    def hex_to_radians(self, hex):
        rad_span = (self.radian_range[1] - self.radian_range[0])
        hex_span = (self.position_range[1] - self.position_range[0])
        rad = self.radian_range[0] + (hex - self.position_range[0]) * rad_span / hex_span
        return rad
    def hex_from_radians(self, rad):
        rad_span = (self.radian_range[1] - self.radian_range[0])
        hex_span = (self.position_range[1] - self.position_range[0])
        hex = self.position_range[0] + int((rad - self.radian_range[0]) * hex_span / rad_span)
        return hex

class Controller:
    def __init__(self):
        #self.connection = Connection()
        self.connection = Simulation()
        self.gripper =  Servo(self, jid=6, sid=1)
        self.hand  =    Servo(self, jid=5, sid=2)
        self.wrist  =   Servo(self, jid=4, sid=3)
        self.elbow =    Servo(self, jid=3, sid=4)
        self.shoulder = Servo(self, jid=2, sid=5)
        self.base =     Servo(self, jid=1, sid=6)
        self.joints = [ self.base, self.shoulder, self.elbow, self.wrist, self.hand, self.gripper ]
        self.product_id = 0x5750
        self.vendor_id = 0x0483
    def connect(self):
        self.connection.connect(self.product_id, self.vendor_id)
    def disconnect(self):
        self.connection.close()
    def home(self):
        for joint in self.joints:
            joint.set_position_radians(0, 5)

if __name__ == "__main__":
    # test code
    arm = Controller()
    arm.connect()
    #arm.reset_servos()
    #print("END RESET")
    #clock.sleep(3)

    # t = 0
    # while t <= 1.0:
    #     while arm.wrist.is_moving():
    #         continue
    #     arm.wrist.set_position(t, 8)
    #     t += 0.2
    
    def qToString(q):
        """ Takes an nparray of radian angles, returns a nice string showing both hex and degrees """
        d0 = q[0] * 180/np.pi
        h0 = arm.joints[0].hex_from_radians(q[0])
        d1 = q[1] * 180/np.pi
        h1 = arm.joints[1].hex_from_radians(q[1])
        d2 = q[2] * 180/np.pi
        h2 = arm.joints[2].hex_from_radians(q[2])
        d3 = q[3] * 180/np.pi
        h3 = arm.joints[3].hex_from_radians(q[3])
        d5 = q[4] * 180/np.pi
        h5 = arm.joints[4].hex_from_radians(q[5])
        return "[ b: %4d or %4.0d°  s: %4d or %4.0d°  e: %4d or %4.0d°  e: %4d or %4.0d°  w: %4d or %4.0d° ]" % (
                h0, d0, h1, d1, h2, d2, h3, d3, h5, d5)

    q_current = np.array([joint.get_position_radians() for joint in arm.joints])
    end_pos = calculate_end_pos(q_current)
    print(f"servos = {qToString(q_current)} so end_pos = {cartesianToString(end_pos)}")

    dest_coords = None
    # - If called with no parameters, it tries to move to a hardcoded position.
    # - If called with parameters like "goto 0.2 0.1 0.3" it will try to move to
    #   those coordinates. For any of the coordinates, you can use '_' to mean
    #   'current position'.
    # - If called with parameters like "move 0.0 0.0 -0.1" it will try to move
    #   relative to the current coordinates by the given deltas. here, '_' is
    #   the same as 0.0 as a delta.
    if len(sys.argv) == 1:
        dest_coords = [0.1, 0.1, 0.2]
    elif len(sys.argv) == 5 and sys.argv[1] == "goto":
        x = end_pos[0] if sys.argv[2] == '_' else float(sys.argv[2])
        y = end_pos[1] if sys.argv[3] == '_' else float(sys.argv[3])
        z = end_pos[2] if sys.argv[4] == '_' else float(sys.argv[4])
        dest_coords = [x, y, z]
    elif len(sys.argv) == 5 and sys.argv[1] == "move":
        dx = 0.0 if sys.argv[2] == '_' else float(sys.argv[2])
        dy = 0.0 if sys.argv[3] == '_' else float(sys.argv[3])
        dz = 0.0 if sys.argv[4] == '_' else float(sys.argv[4])
        dest_coords = [end_pos[0]+dx, end_pos[1]+dy, end_pos[2]+dz]
    elif len(sys.argv) == 2 and sys.argv[1] == "hold":
        dest_coords = None
    else:
        print("Bad arguments. Try these examples:")
        print("  goto 0.1 0.1 0.25   # go to the specified x,y,z coordinates ")
        print("  goto _ _ 0.25       # go to the specified z coordinate, leave x and y alone")
        print("  move 0.1 _ -0.05    # move by the specfied dx and dz, leave y alone")
        print("  hold                # just monitor the current position")
        sys.exit(0)

    if dest_coords:
        cart_target = np.transpose(np.array(dest_coords))
        nearby, moved = nearest_reachable_point(cart_target)
        if moved:
            print(f"NOTE: {cartesianToString(cart_target)} is outside the reach of the arm.")
            print(f"NOTE: {cartesianToString(nearby)} will be targetted instead.")
            cart_target = nearby

    preverr = 100
    stall = 0
    attempted_reset = False
    while True:
        clock.sleep(0.1)
        q_current = np.array([joint.get_position_radians() for joint in arm.joints])
        end_pos = calculate_end_pos(q_current)
        # print(f"servos = {qToString(q_current)} so end_pos = {cartesianToString(end_pos)}")
        if dest_coords:
            err = np.linalg.norm(cart_target - end_pos)
            if err < 0.001:
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
                    continue
            else:
                print(f"progress stalled (attemped {stall} of 20)...")
                stall += 1
            preverr = err
            q_delta = calculate_joint_angles_delta(q_current, cart_target)
            q_new = np.array(q_current) + q_delta
            print(f"target = {qToString(q_new)} end_pos = {cartesianToString(end_pos)} err = {err*1000} mm")
            # print(f"q_delta: {q_delta}")
            h6 = arm.joints[0].set_position_radians(q_new[0], 5)
            h5 = arm.joints[1].set_position_radians(q_new[1], 5)
            h4 = arm.joints[2].set_position_radians(q_new[2], 5)
            h3 = arm.joints[3].set_position_radians(q_new[3], 5)
            h2 = arm.joints[4].set_position_radians(q_new[4], 5)
            # print(f"{h6} {h5} {h4} {h3} {h2}")

    #arm.home()
    arm.disconnect()
