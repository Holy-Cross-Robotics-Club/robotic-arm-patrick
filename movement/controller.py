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
    def set_position_hex(self, pos, time):
        pos = self.__swap_halves(pos)
        pos = self.__split_bytes(pos)
        #move gripper: [85, 85, # bytes, command, # servos, time(2), time(1), servo id, pos(2), pos(1)]
        self.controller.connection.write_out([85, 85, 7, 3, 1, 0, time, self.id, pos[0], pos[1]])
    def set_position(self, t, time):
        pos = self.__normalize_pos(t)
        self.set_position_hex(pos, time)
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
        self.position_range = [0x00A0, 0x02C0] # these are hard-coded

class ServoWrist(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 2
        self.position_range = [0x0080, 0x0350]

class ServoElbow(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 3
        self.position_range = [0x0040, 0x0400]

class ServoBase(Servo):
    def __init__(self, controller):
        super().__init__(controller)
        self.id = 6
        self.position_range = [0x0010, 0x0400]

class Controller:
    def __init__(self):
        self.connection = Connection()
        self.gripper = ServoGripper(self) # id 6
        self.elbow = ServoElbow(self) # id 3
        self.wrist = ServoWrist(self) # id 2
        self.base = ServoBase(self) # id 1
        self.product_id = 0x5750
        self.vendor_id = 0x0483
    def reset_servos(self):
        self.gripper.set_position(0.5,3)
        self.wrist.set_position(0.5,3)
        self.elbow.set_position(0.5,3)
        self.base.set_position(0.5,3)
    def connect(self):
        self.connection = Connection()
        self.connection.connect(self.product_id, self.vendor_id)
    def disconnect(self):
        self.connection.close()

if __name__ == "__main__":
    import time
    # test code
    controller = Controller()
    controller.connect()
    controller.reset_servos()
    time.sleep(2)

    t = 0
    while t <= 1.0:
        controller.gripper.set_position(t, 3) # move base in steady increments
        controller.base.set_position(t, 3) # two servos can move at the same time!
        controller.wrist.set_position(1-t, 3) # this one too!
        controller.elbow.set_position(t,3)
        t += 0.1
        time.sleep(2.5)
    
    controller.reset_servos()
    controller.disconnect()