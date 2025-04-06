import hid

# This code provides two classes:
# - Connection is uused to communicate over USB to a robot arm. This is used
#   by other files to send motion commands or queries to the actual robot.
# - DeadEnd does nothing, but provides the same API. This can be used to plug
#   into code that needs a Connection object when you don't actualy want to send
#   any commands annywhere.
#
# The data sent to/from the robot needs to be in a very specific format,
# following the robot communication protocol. But this code only does some basic
# checks on the data, e.g. parsing the first few bytes of a response to see how
# large a data packet is. It is up to callers to ensure you send the right byte
# sequences in the right order.
#
# The controller.py code is designed to work with either Connection, Simulation,
# DeadEnd, or a combination of two of these.

class Connection:
    """ Simple wrapper for hidapi.
    Intended to read and write OUT/IN reports easily.
    """
    def __init__(self):
        self.connection = None
        self.info = None
    def connect(self, pid, vid):
        """ 
        Connect to a USB HID device with the specified product ID (pid) and vendor ID (vid). 
        Enumerates all connected USB HID devices, matches against the given PID and VID. if found, opens a connection to the device.
        """
        devs = hid.enumerate()
        for d in devs:
            if d['product_id'] == int(pid) and d['vendor_id'] == int(vid):
                self.info = d
                self.connection = hid.device()
                self.connection.open_path(self.info['path'])
        if self.connection is None: raise AssertionError("USB connection for arm not detected. Are you sure it's plugged in and turned on?")
        else: print(f"Successfully conntected to {self.info['product_string']}.")
    def close(self):
        if self.connection is None: return
        self.connection.close()
        print(f"Closed {self.info['product_string']}.")
    def write_out(self, m):
        # TODO: do some sanity checks
        cnt = self.connection.write(m)
        if cnt != len(m):
            print(f"USB ERROR: result={err} after writing {len(m)} bytes {m}")
    def read_in(self, cmd, return_param_size):
        tot_size = 4 + return_param_size # 0x55 0x55 len cmd ... followed by return_params
        data = self.connection.read(4 + return_param_size)
        if not data:
            print("USB ERROR: expecting cmd results, got nothing")
            return None
        if len(data) < 4:
            print(f"USB ERROR: expecting at least 4 bytes of results, got {len(data)} bytes")
            return None
        if data[0] != 85 or data[1] != 85:
            print(f"USB ERROR: results have bad header, got {data[0]} {data[1]}")
            return None
        if data[2] != tot_size-2 or data[3] != cmd:
            print(f"USB ERROR: results expected len {tot_size-2} and cmd {cmd}, but got {data[2]} and {data[3]}")
            return None
        if len(data) != tot_size:
            print(f"USB ERROR: results expected {tot_size} total bytes, got {len(data)} bytes")
            return None
        return data[4:]

class DeadEnd:
    """ An alternative implementation of Connection that does nothing. """
    def __init__(self):
        pass
    def connect(self, pid, vid):
        pass
    def close(self):
        pass
    def write_out(self, m):
        pass
    def read_in(self, cmd, num_bytes):
        return None
