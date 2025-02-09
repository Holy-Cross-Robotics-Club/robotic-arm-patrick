import socket
import sys

# The backend python/html/js simulation is contacted via TCP at this address.
arm_addr = ('localhost', 8002)

debug = False

# Sends a message over TCP, with a simple header and framing. First a 3-byte
# header (0xAB, 0xCD, 0xEF) is sent, then a 1-byte length, then the message.
def sendmsg(conn, msg):
    if len(msg) > 255:
        raise AssertionError(f"Error, message len {len(msg)} too large")
    conn.send(bytes([ 0xAB, 0xCD, 0xEF, len(msg)]))
    if debug:
        print('sending: [' + ', '.join(f'0x{b:02x}' for b in msg) + ']')
    conn.send(msg)


# Receives and returns exactly the given number of bytes from a TCP socket.
# If any failures are encountered, returns None instead.
def recvexactly(conn, length):
    received = 0
    buf = b''
    while received < length:
        data = conn.recv(length - received)
        if data is None:
            return None
        buf += data
        received = len(buf)
    return buf

# Receives a message over TCP as sent by the sendmsg() function above, removing
# the header and framing, and returning just the message body.
def recvmsg(conn):
    data = recvexactly(conn, 4)
    if not data:
        return None
    if data[0] != 0xAB or data[1] != 0xCD or data[2] != 0xEF:
        print(f"Error: tcp message malformed")
        return None
    mlen = data[3]
    if mlen == 0:
        return bytes()
    data = recvexactly(conn, mlen)
    if not data:
        return None
    if debug:
        print('recieved: [' + ', '.join(f'0x{b:02x}' for b in data) + ']')
    return data

# Same as sendmsg(), but uses asyncio writer.
async def async_sendmsg(writer, msg):
    if len(msg) > 255:
        raise AssertionError(f"Error, message len {len(msg)} too large")
    writer.write(bytes([ 0xAB, 0xCD, 0xEF, len(msg)]))
    if debug:
        print('sending: [' + ', '.join(f'0x{b:02x}' for b in msg) + ']')
    writer.write(msg)
    await writer.drain()

# Same as recvmsg(), but uses asyncio reader.
async def async_recvmsg(reader):
    data = await reader.readexactly(4)
    if not data:
        return None
    if data[0] != 0xAB or data[1] != 0xCD or data[2] != 0xEF:
        print(f"Error: tcp message malformed")
        return None
    mlen = data[3]
    if mlen == 0:
        return bytes()
    data = await reader.readexactly(mlen)
    if not data:
        return None
    if debug:
        print('recieved: [' + ', '.join(f'0x{b:02x}' for b in data) + ']')
    return data

# A replacement for Connection, but uses TCP to connect to a python/html/js
# simulation of the robot arm, rather than using HID-USB to talk to a real arm.
class Simulation:
    """ Same API as Connection, but uses TCP instead of HID-USB.
    """
    def __init__(self):
        self.connection = None
    def connect(self, pid, vid, fatal=True):
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.connect(arm_addr)
            print(f"Successfully conntected to simulation at {arm_addr}.")
        except:
            if fatal:
                print("ERROR: Could not connect to browser-based simulation.")
                print("Are you sure you are running the simulation server?")
                print("To start the simulator:")
                print("  1. open a new terminal window")
                print("  2. change into the robotic-arm-patrick directory")
                print("  3. activate the virtual env, for example: source ~/xarm-python-venv/bin/activate")
                print("  4. run the server, for example: ./simulation/server.py")
                sys.exit(1)
            else:
                self.connection = None
                raise ConnectionError(f"Could not connect to simulation at {arm_addr}.")
    def close(self):
        if self.connection is None: return
        self.connection.close()
        print(f"Closed TCP connection to simulation at {arm_addr}")
    def write_out(self, m):
        sendmsg(self.connection, bytes(m))
    def read_in(self, cmd, num_bytes):
        data = recvmsg(self.connection)
        if not data:
            return None
        if len(data) < 4:
            print(f"Error, not enough data for header, length tag, and cmd byte")
            return None
        if data[0] == 85 and data[1] == 85 and data[2] == num_bytes and data[3] == cmd:
            return data[4:]
        else:
            print(f"Error, wanted 2+{num_bytes} and cmd {cmd} but have {len(data)} bytes of data")
            return None

