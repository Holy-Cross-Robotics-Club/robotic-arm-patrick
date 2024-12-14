from connection import Connection
from kinematics import *
import time as clock

class Servo:
    """Acts as an interface to control and read from the servos.
    Works with connection.py to parse requests to HID-USB format.
    """
    def __init__(self, controller):
        self.position_range = [None, None]
        self.id = None
        self.__is_moving = False
        self.__target_position = None
        self.controller = controller
        self.curr_set_pos = 0.5
    def get_position_hex(self):
        self.controller.connection.write_out([85, 85, 4, 21, 1, self.id])
        result = self.controller.connection.read_in(21, 6)
        hex = (result[3] * 256 + result[2]) # returns hex
        return hex
    def get_position_radians(self):
        return self.__normalized_to_radians(self.get_position())
    def get_position(self):
        hex = self.get_position_hex()
        t = self.__normalized_from_hex(hex)
        #print(f"The respective read-in value is {hex} (hex) and {t} (normalized).")
        return t
    def is_moving(self):
        if self.curr_set_pos is None:
            return False
        diff = abs(self.get_position() - self.curr_set_pos)
        print(f"{self.get_position()} (GET) and {self.curr_set_pos} (SET)")
        return False if diff <= 0.03 else True
    def set_position_hex(self, pos, time):
        clock.sleep(0.03)
        self.curr_set_pos = self.__normalized_from_hex(pos)
        print(f"{self.id}: Set position is now {self.curr_set_pos}")
        pos = self.__swap_halves(pos)
        pos = self.__split_bytes(pos)
        #move gripper: [85, 85, # bytes, command, # servos, time(2), time(1), servo id, pos(2), pos(1)]
        self.controller.connection.write_out([85, 85, 8, 3, 1, 0, time, self.id, pos[0], pos[1]])
    def set_position(self, t, time):
        t = int(t * 100) / 100 # round to nearest hundredths
        pos = self.__normalize_pos(t)
        #print(f"Position set to: {t} (normalized) and {pos} (hex).")
        self.set_position_hex(pos, time)
    def set_position_radians(self, rad, time):
        t = self.__normalized_from_radians(rad)
        self.set_position(t, time)
    def set_position_min(self, time):
        self.set_position(0, time)
    def set_position_max(self, time):
        self.set_position(1, time)
    def off(self):
        pass
    def __swap_halves(self, val):
        left_half = (val & 0xFF00) >> 8
        right_half = (val & 0x00FF)
        return (right_half << 8) | left_half
    def __split_bytes(self, val):
        upper_byte = (val >> 8) & 0xFF
        lower_byte = val & 0xFF
        return upper_byte, lower_byte
    def __normalize_pos(self, t):
        return int(self.position_range[0] + t * (self.position_range[1] - self.position_range[0]))
    def __normalized_from_hex(self, hex):
        t = (hex - self.position_range[0])/(self.position_range[1] - self.position_range[0])
        return int(t * 100) / 100 # round to nearest hundredths
    def __normalized_to_radians(self, t):
        rad = self.radian_range[0] + t * (self.radian_range[1] - self.radian_range[0])
        #shifted_rad = rad + (np.pi / 2)
        shifted_rad = rad
        return shifted_rad
    def __normalized_from_radians(self, rad):
        #shifted_rad = rad - (np.pi / 2)
        shifted_rad = rad
        t = (shifted_rad - self.radian_range[0]) / (self.radian_range[1] - self.radian_range[0])
        return t

class ServoGripper(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 1
        self.position_range = [0x00A0, 0x02C0] # these are hard-coded
        self.radian_range = [-1 * np.pi / 2, np.pi / 2]

class ServoWrist(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 2
        #self.position_range = [0x0080, 0x0350]
        self.position_range = [200, 980]
        #self.radian_range = [-1 * np.pi, np.pi]
        self.radian_range = [0, np.pi]

class ServoElbow(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 3
        #self.position_range = [0x003a, 0x039f]
        self.position_range = [140, 880]
        #self.radian_range = [-1 * 3 * (np.pi)/4, 3 * (np.pi)/4]
        self.radian_range = [0, np.pi]

class ServoElbow2(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 4
        #self.position_range = [0x0004, 0x03e1]
        self.position_range = [130, 870]
        #self.radian_range = [-1 * 3 * (np.pi)/4, 3 * (np.pi)/4]
        self.radian_range = [0, np.pi]

class ServoShoulder(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 5
        #self.position_range = [0x0090, 0x0370]
        self.position_range = [140, 880]
        #self.radian_range = [-1 * np.pi/2, np.pi/2]
        self.radian_range = [0, np.pi]

class ServoBase(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 6
        #self.position_range = [0x0000, 0x03f0]
        self.position_range = [90, 845]
        #self.radian_range = [-1 * np.pi, np.pi]
        self.radian_range = [0, np.pi]

class Controller:
    def __init__(self):
        self.connection = Connection()
        self.gripper = ServoGripper(self) # id 6
        self.shoulder = ServoShoulder(self) # id 5
        self.elbow2 = ServoElbow2(self) # id 4
        self.elbow = ServoElbow(self) # id 3
        self.wrist = ServoWrist(self) # id 2
        self.base = ServoBase(self) # id 1
        self.product_id = 0x5750
        self.vendor_id = 0x0483
    def reset_servos(self):
        self.gripper.set_position(0.5,3)
        self.wrist.set_position(0.5,3)
        self.elbow2.set_position(0.5,3)
        self.elbow.set_position(0.5, 3)
        self.shoulder.set_position(0.5, 3)
        self.base.set_position(0.5,3)
    def connect(self):
        self.connection = Connection()
        self.connection.connect(self.product_id, self.vendor_id)
    def disconnect(self):
        self.connection.close()

if __name__ == "__main__":
    # test code
    controller = Controller()
    controller.connect()
    #controller.reset_servos()
    #print("END RESET")
    #clock.sleep(3)

    # t = 0
    # while t <= 1.0:
    #     while controller.wrist.is_moving():
    #         continue
    #     controller.wrist.set_position(t, 8)
    #     t += 0.2
    
    base = controller.base
    shoulder = controller.shoulder
    elbow2 = controller.elbow2
    elbow = controller.elbow
    wrist = controller.wrist
    gripper = controller.gripper

    cart_target = np.transpose(np.array([0.2, 0, 0.15]))

    while True:
        clock.sleep(0.5)
        q_current = np.array([base.get_position_radians(), shoulder.get_position_radians(), elbow2.get_position_radians(), elbow.get_position_radians(), 0, wrist.get_position_radians()])
        q_delta = calculate_joint_angles_delta(q_current, cart_target)
        q_new = np.array(q_current) + q_delta
        print(f"q_delta: {q_delta}")
        base.set_position_radians(q_new[0], 5)
        shoulder.set_position_radians(q_new[1], 5)
        elbow2.set_position_radians(q_new[2], 5)
        elbow.set_position_radians(q_new[3], 5)
        wrist.set_position_radians(q_new[5], 5)

    print(q_new)

    #controller.reset_servos()
    controller.disconnect()