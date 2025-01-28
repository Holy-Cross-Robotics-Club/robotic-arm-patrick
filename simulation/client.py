#!/usr/bin/env python3

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
