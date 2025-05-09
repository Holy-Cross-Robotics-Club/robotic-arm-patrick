#!/usr/bin/env python3

# This is a standalone program, implementing a simulation of a virtual robotic
# arm, and providing an API to control the arm from other programs. 
#
# See README.md in this directory for details on installing and running. The
# programs in the movement folder will start the server as needed, or give
# instructions on how to start it.
# 
# The code below has several parts:
#
#  - A "physics" simulation of the xArm itself. Basically, we just keep a set of
#    values corresponding to joint angles. These are updated according to a very
#    primitive physics/motor simulation. For example, if some angle is currently
#    X degrees, but the user wants it to be Y degress, then X will be adjusted
#    incrementally to be closer to Y. The physics could be improved, modeling
#    acceleration, etc., but for now, just a simple animation is enough.
#
#  - A "controller" simulation of the xArm's control board. This code tries to
#    mimic the behavior of the controller board on the xArm, accepting the same
#    movement and query commands, and sending back the same responses as the
#    real xArm. This tries to use essentially the same protocol, byte-for-byte,
#    as the real arm.
#
# - A "browser-facing" web-server, allowing for an in-browser html/javascript
#   visualization of the arm's state. There are three parts to this:
#
#   1. When the browser fetches "/index.html" or "/arm.js", or other assets,
#      over HTTP on port 8000, those files are sent back to the browser.
#
#   2. The asset "/parameters.json" is special. For this one, a special json
#      object is sent to the browser containing all the DH parameters for the
#      robot arm.
#
#   3. The resulting page uses javascript to also open a websocket from the
#      browser back to this server on port 8001. This connection is used to
#      query and update the joint angles. In other words, it allows updates to
#      the joint angles to be sent back to the browser, or for user input to
#      come from the browser back to the simulation to control the joins.
#
# - A "client-facing" tcp-server, allowing for other python code to connect and
#   either query or update the joint angles. The protocol used here is nearly
#   identical to the protocol python would use to interact with the real xArm,
#   except it doesn't use USB/USB-HID, it uses a TCP socket instad.


import asyncio
from aiohttp import web
import websockets
import socket
import threading
import signal
import sys
import os
import traceback
import time
import webbrowser
from simulation import async_recvmsg, async_sendmsg
import parameters

debug = False
running = True

http_port = 8000
ws_port = 8001
arm_port = 8002

# The DH parameter table and current angle values are stored in the following
# lists. These come from parameters.py, but they might look like:
# servo    -      1     2     3     4     5     6
# joint    -      5     4     3     2     1     0
# cmin = [ 0,   155,  120,   70,   10,  144,    0 ]
# cmax = [ 0,   666,  880,  930,  990,  880, 1000 ]
# cval = [ 0,   432,  488,  492,  498,  512,  504 ]
# ctgt = [ 0,   432,  488,  492,  498,  512,  504 ]
#
# Note: position 0 in the array is unused, since there is no server number 0. We
# also reverse the arrays in parameter.py, so they are ordered by servo number
# here instead of by joint number.

# cmin and cmax are the range of valid click values for each joint.
cmin = [ 0 ] + [ parameters.joint_min_clk[i] for i in [5, 4, 3, 2, 1, 0] ]
cmax = [ 0 ] + [ parameters.joint_max_clk[i] for i in [5, 4, 3, 2, 1, 0] ]
# cval is the current position of each joint, in clicks
cval = [ int((lo + hi) / 2) for lo, hi in zip(cmin, cmax) ]
# ctgt is the desired position of each joint, in clicks, e.g. as set by the
# motion commands sent by the user.
ctgt = cval.copy()
# cstp controls speed, as the maximum number of clicks to move a motor in each
# step of the physics simulation, as determined by the speed parameter of the
# motion commands sent by the user.
cstp = [ 0 ] * 7
# rmin and rmax are the range of reachable angles, in radians, for each joint.
rmin = [ 0 ] + [ parameters.joint_min_rad[i] for i in [5, 4, 3, 2, 1, 0] ]
rmax = [ 0 ] + [ parameters.joint_max_rad[i] for i in [5, 4, 3, 2, 1, 0] ]

time_step = 0.1 # seconds, for physics loop and animation

# make a json bundle with parameters to give to javascript in browser
json = '{ "cmin" : %s, "cmax" : %s, "rmin": %s, "rmax": %s, "d1": %s, "a2": %s, "a3": %s, "d5": %s }' % (
        cmin, cmax, rmin, rmax, parameters.d1, parameters.a2, parameters.a3, parameters.d5)

async def physics_loop():
    while running:
        msg = bytearray()
        count = 0
        for j in range(1, 7):
            delta = ctgt[j] - cval[j]
            amt = max(-cstp[j], min(delta, cstp[j]))
            if amt != 0:
                count += 1
                cval[j] += amt
                msg.extend([ j, cval[j] & 0xff, (cval[j] >> 8) & 0xff])
                if debug:
                    print(f"gently moving servo {j} by {amt} clicks")
        if count > 0:
            print("adjust positions: " + ' '.join(f'{b:4d}' for b in cval[1:]))
            msg.insert(0, 0) # t_hi
            msg.insert(0, 0) # t_lo
            msg.insert(0, count) # nservos
            msg.insert(0, 3) # cmd
            msg.insert(0, 1+len(msg)) # len
            msg.insert(0, 0x55) # hdr_hi
            msg.insert(0, 0x55) # hdr_lo
            await publish(bytes(msg))
        # else:
        #     print("ctgt = " + str(ctgt))
        #     print("cval = " + str(cval))
        await asyncio.sleep(time_step)
    print("Physics loop done")

async def process_usb_report(msg):
    if len(msg) < 3:
        print('Report too short.')
        return
    if msg[0] != 0x55 or msg[1] != 0x55:
        print('Header invalid, should be [ 0x55, 0x55 ].')
        return
    mlen = msg[2]
    if mlen != len(msg) - 2:
        print(f'WARNING: length byte is {mlen} but report has {len(msg)-2} bytes of data excluding the 2-byte header')
    cmd = msg[3] if 3 < len(msg) else 0
    if cmd == 3: # move servo(s)
        if debug:
            print("command: move servo(s)")
        nservos = msg[4] if 4 < len(msg) else 0
        if debug:
            print(f"  nservos = {nservos}")
        if mlen != 5 + nservos*3: # 5 because len + cmd + nservo + t_lo + t_hi
            print(f'  WARNING: nservos was {nservos} but that requires length to be {5 + nservos*3}')
        t_lo = msg[5] if 5 < len(msg) else 0
        t_hi = msg[6] if 6 < len(msg) else 0
        t = (t_hi << 8) | t_lo
        if debug:
            print(f"  time = {t}")
        if t == 0:
            t = 1
        for i in range(nservos):
            axis = msg[2+5+3*i+0] if 2+5+3*i+0 < len(msg) else 0
            p_lo = msg[2+5+3*i+1] if 2+5+3*i+1 < len(msg) else 0
            p_hi = msg[2+5+3*i+2] if 2+5+3*i+2 < len(msg) else 0
            p = (p_hi << 8) | p_lo
            if axis < 1 or axis > 6:
                print(f'  ERROR: no such axis {axis}')
                continue
            if p != ctgt[axis]:
                if debug:
                    print(f"  move axis {axis} from {cval[axis]} towards position {p}")
                ctgt[axis] = max(cmin[axis], min(cmax[axis], p))
                cstp[axis] = int(abs(ctgt[axis] - cval[axis])/t * time_step * 1000)
                print("target positions: " + ' '.join(f'{b:4d}' for b in ctgt[1:]))
            elif debug:
                print(f"  axis {axis} already at position {p}")
        # await publish(msg);
        return
    elif cmd == 21: # query position(s)
        if debug:
            print("command: query position(s)")
        nservos = msg[4] if 4 < len(msg) else 0
        if debug:
            print(f"  nservos = {nservos}")
        if mlen != 3 + nservos*1: # 3 because len + cmd + nservo
            print(f'  WARNING: nservos was {nservos} but that requires length to be {3 + nservos*1}')
        resp = bytearray()
        count = 0
        for i in range(nservos):
            axis = msg[2+3+i] if 2+3+i < len(msg) else 0
            if axis < 1 or axis > 6:
                print('  ERROR: no such axis')
                continue
            count += 1
            resp.extend([ axis, cval[axis] & 0xff , (cval[axis] >> 8) & 0xff ])
        resp.insert(0, count) # nservos
        resp.insert(0, 21) # cmd
        resp.insert(0, 1+len(resp)) # len
        resp.insert(0, 0x55) # hdr_hi
        resp.insert(0, 0x55) # hdr_lo
        return bytes(resp)

qs = []

async def subscribe():
    q = asyncio.Queue()
    qs.append(q)
    # insert move servos command to set the animation initial position
    msg = bytearray()
    msg.append(0x55) # hdr_lo
    msg.append(0x55) # hdr_hi
    msg.append(5+6*3) # len
    msg.append(3) # cmd
    msg.append(6) # nservo
    msg.append(0) # t_lo
    msg.append(0) # t_hi
    for j in range(1, 7):
        msg.extend([ j, cval[j] & 0xff, (cval[j] >> 8) & 0xff])
    await q.put(bytes(msg))
    return q

def unsubscribe(q):
    qs.remove(q)

async def publish(data):
    for q in qs:
        await q.put(data)

# WebSocket handler
async def ws_handler(websocket):
    q = await subscribe()
    try:
        while running:
            receive_task = asyncio.create_task(websocket.recv())
            queue_task = asyncio.create_task(q.get())

            done, pending = await asyncio.wait(
                [receive_task, queue_task],
                return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            for task in done:
                try:
                    result = task.result()
                    if task == receive_task:
                        msg = result
                        if len(msg) != 3:
                            print(f"invalid websocket message, len = {len(msg)}")
                            continue
                        axis = msg[0]
                        p_lo = msg[1]
                        p_hi = msg[2]
                        p = (p_hi << 8) | p_lo
                        if debug:
                            print(f"animation moved servo {axis} to position {p}")
                        if axis <= 0 or axis > 6:
                            print("invalid servo number")
                            continue
                        ctgt[axis] = cval[axis] = max(cmin[axis], min(cmax[axis], p))
                    else:
                        # Send queued message
                        if not result:
                            break
                        data = result
                        await websocket.send(data)
                except Exception as e:
                    print(f"Error: {e}")
                    return
    except websockets.ConnectionClosed:
        pass
    except:
        if running:
            traceback.print_exc()
    finally:
        unsubscribe(q)

async def handle_arm_connection(reader, writer):
    try:
        while running:
            data = await async_recvmsg(reader)
            if not data:
                print(f"Lost arm control connection to {writer.get_extra_info('peername')}")
                break
            if debug:
                print('ARM CONTROL: ' + ' '.join(f'0x{b:02x}' for b in data))
            resp = await process_usb_report(data)
            if resp is not None:
                await async_sendmsg(writer, resp)
    except:
        if running:
            traceback.print_exc()
    finally:
        writer.close()
        await writer.wait_closed()
        print(f"Closed arm control connection to {writer.get_extra_info('peername')}")

# handle http file requests
async def handle_file(request, webdir, filename=None):
    if filename is None:
        filename = request.match_info['filename']
    filepath = os.path.join(webdir, filename)
    if not os.path.isfile(filepath):
        return web.Response(status=404)
    return web.FileResponse(filepath)

async def handle_json(request):
    return web.Response(text=json)

async def shutdown():
    global running, http_runner, ws_server, arm_server
    if running:
        running = False
        print("Cleaning up queue")
        await publish(None)
        print("Cleaning up http server")
        await http_runner.cleanup()
        print("Cleaning up websocket server")
        ws_server.close()
        await ws_server.wait_closed()
        print("Cleaning up arm server")
        arm_server.close()
        await arm_server.wait_closed()
        print("Stopping loop")
        loop = asyncio.get_running_loop()
        loop.stop()

async def main(webdir):
    global running, http_runner, ws_server, arm_server

    app = web.Application()
    app.router.add_get('/', lambda r: handle_file(r, webdir, 'arm.html'))
    app.router.add_get('/parameters.json', lambda r: handle_json(r))
    app.router.add_get('/{filename}', lambda r: handle_file(r, webdir))

    # (1) Start http server
    http_runner = web.AppRunner(app)
    await http_runner.setup()
    site = web.TCPSite(http_runner, 'localhost', http_port)
    await site.start()
    arm_url = f"http://localhost:{http_port}/arm.html"
    print(f"HTTP server running on ${arm_url}")

    # (2) Start websocket server
    ws_server = await websockets.serve(ws_handler, "", ws_port)
    # print(f"WebSocket server running on ws://localhost:{ws_port}")

    # (3) Start arm control server
    arm_server = await asyncio.start_server(handle_arm_connection, 'localhost', arm_port)
    # print(f"xArm Control server running on localhost:{arm_port}")

    # (4) Start physics loop
    asyncio.create_task(physics_loop())

    webbrowser.open(arm_url)

    print("Simulation is ready, should be open in browser. Open above URL if not.")
    print("Leave this running, or hit Control-C to stop the simulation.")

    # Keep running everything
    try:
        await asyncio.Future()  # run forever
    finally:
        await shutdown()

def signal_handler(signum, frame):
    print("\n=============== Shutdown requested... ====================")
    loop = asyncio.get_running_loop()
    loop.create_task(shutdown())

if __name__ == "__main__":
    webdir = os.path.dirname(os.path.realpath(__file__))

    signal.signal(signal.SIGINT, signal_handler)
    try:
        asyncio.run(main(webdir))
    except:
        if running:
            traceback.print_exc()
