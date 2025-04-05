#!/usr/bin/env python3

# This is a standalone program, implementing a client to interact with the
# simulation server. It is mostly for testing, to ensure the simulation server
# works as intended.
#
# This program has a very basic, low-level user interface. The user types lists
# of byte values (in either hex or decimal notation). These are sent to the
# simulation server over a tcp socket, and any responses are printed. This is
# much like other code might send USB packets to the real arm, but with the user
# typing all data directly. So if you type the right sequence of bytes,
# representing movement or query commands, the simulated arm will move and/or
# send back responses about where the joints are. 
#
# The specific bytes you need to type here are the same ones the real xArm
# requires for its protocol.

import socket
import sys
import time
import threading
from simulation import recvmsg, sendmsg

arm_port = 8002

running = True

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def input_thread():
    global running
    global conn
    while running:
        try:
            line = input()
        except:
            print("Input cancelled, exiting")
            time.sleep(0.5)
            running = False
            conn.close()
            return
        try:
            line = line.replace(',', ' ').replace('[', ' ').replace(']', ' ').strip()
            if not line:
                continue
            values = [int(x, 0) for x in line.split()]
            sendmsg(conn, bytes(values))
        except ValueError:
            print("Invalid input")

try:
    conn.connect(('localhost', arm_port))
    
    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    while running:
        try:
            data = recvmsg(conn)
            if data:
                print(' '.join(f'0x{b:02x}' for b in data))
        except:
            if running:
                print("Lost connection")
                running = False
                conn.close()

finally:
    conn.close()
