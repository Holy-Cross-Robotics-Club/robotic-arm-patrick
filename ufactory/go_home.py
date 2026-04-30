#!/usr/bin/env python3
"""
Description: Return the UFACTORY xArm 6 to its home position.

Run from the ufactory/ directory (next to robot.conf):
    python3 go_home.py

Or pass the IP directly:
    python3 go_home.py 192.168.7.2
"""

import os
import sys

from xarm.wrapper import XArmAPI


if len(sys.argv) >= 2:
    ip = sys.argv[1]
    print(f"From command line, xArm ip = {ip}")
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        conf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot.conf')
        parser.read(conf_path)
        ip = parser.get('xArm', 'ip')
        print(f"From robot.conf, xArm ip = {ip}")
    except Exception:
        ip = input('Please input the xArm ip address: ')
        if not ip:
            print('input error, exit')
            sys.exit(1)


arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

print("Returning to home position...")
arm.move_gohome(wait=True)
arm.disconnect()
print("Done.")