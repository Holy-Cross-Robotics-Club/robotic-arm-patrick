from connection import Connection

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
    def get_position(self):
        pass
    def is_moving(self):
        return self.__is_moving
    def set_position(self, t, time):
        pos = self.__normalize_pos(t)
        pos = self.__swap_halves(pos)
        pos = self.__split_bytes(pos)
        #move gripper: [85, 85, # bytes, command, # servos, time(2), time(1), servo id, pos(2), pos(1)]
        self.controller.connection.write_out([85, 85, 7, 3, 1, 0, time, self.id, pos[0], pos[1]])
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

class ServoGripper(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 1
        self.position_range = [0x00A0, 0x02C0] # these are hard-coded TODO: self calibration?

class ServoBase(Servo):
    def __init__(self, controller):
        super().__init__(controller)

class Controller:
    def __init__(self):
        self.connection = Connection()
        self.gripper = ServoGripper(self)
        self.base = ServoBase(self)
        self.product_id = 0x5750
        self.vendor_id = 0x0483
    def connect(self):
        self.connection.connect(self.product_id, self.vendor_id)
    def disconnect(self):
        self.connection.close()

if __name__ == "__main__":
    # test code
    controller = Controller()
    controller.connect()
    import time
    # controller.gripper.set_position(0, 5) # all the way open
    # time.sleep(2)
    # controller.gripper.set_position(0.5, 5) # in between
    # time.sleep(2)
    # controller.gripper.set_position(1, 5) # all the way closed
    # time.sleep(2)
    # controller.gripper.set_position(0.5, 5)
    t = 0
    while t <= 1.0:
        controller.gripper.set_position(t, 2) # close gripper in steady increments
        t += 0.1
        time.sleep(2)
    controller.disconnect()