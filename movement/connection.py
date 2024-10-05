import hid

class Connection:
    """ Simple wrapper for hidapi.
    Intended to read and write OUT/IN reports easily.
    """
    def __init__(self):
        self.connection = None
        self.info = None
    def connect(self, pid, vid):
        devs = hid.enumerate()
        for d in devs:
            if d['product_id'] == int(pid) and d['vendor_id'] == int(vid):
                self.info = d
                self.connection = hid.device()
                self.connection.open_path(self.info['path'])
        if self.connection is None: raise AssertionError("Could not find a matching device.")
        else:
            print(f"Successfully conntected to {self.info['product_string']}.")
    def close(self):
        if self.connection is None: return
        self.connection.close()
        print(f"Closed {self.info['product_string']}.")
    def write_out(self, m):
        # TODO: do some sanity checks
        self.connection.write(m)
    def read_in(self): #TODO
        pass

if __name__ == "__main__":
    device = Connection()
    pid = 0x5750 # 0x5750
    vid = 0x0483 # 0x0483
    device.connect(pid, vid)
    import time
    device.write_out([85,85,8,3,1,0,5,1,1,1,1,0]) # gripper open grip
    time.sleep(2)
    device.write_out([85,85,8,3,1,0,5,1,255,100,1,0]) # gripper closed grip
    device.close()