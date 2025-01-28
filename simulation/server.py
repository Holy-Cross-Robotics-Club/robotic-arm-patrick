#!/usr/bin/env python3

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
from simulation import async_recvmsg, async_sendmsg

debug = False
running = True

http_port = 8000
ws_port = 8001
arm_port = 8002

#     joint     1     2     3     4     5     6
cmin = [ 0,   160,  128,   58,    4,  144,    0 ]
cmax = [ 0,   704,  848,  926,  992,  880, 1008 ]
cval = [ 0,   432,  488,  492,  498,  512,  504 ]
ctgt = [ 0,   432,  488,  492,  498,  512,  504 ]

acceleration = 100

async def physics_loop():
    while running:
        msg = bytearray()
        count = 0
        for j in range(1, 7):
            delta = ctgt[j] - cval[j]
            amt = max(-acceleration, min(delta, acceleration))
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
        await asyncio.sleep(0.2)
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
async def handle_file(request):
    filename = request.match_info['filename']
    if not os.path.isfile(filename):
        return web.Response(status=404)
    return web.FileResponse(filename)

app = web.Application()
app.router.add_get('/', lambda r: web.FileResponse('index.html'))
app.router.add_get('/{filename}', handle_file)

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

async def main():
    global running, http_runner, ws_server, arm_server

    # (1) Start http server
    http_runner = web.AppRunner(app)
    await http_runner.setup()
    site = web.TCPSite(http_runner, 'localhost', http_port)
    await site.start()
    print(f"HTTP server running on http://localhost:{http_port}")

    # (2) Start websocket server
    ws_server = await websockets.serve(ws_handler, "", ws_port)
    print(f"WebSocket server running on ws://localhost:{ws_port}")

    # (3) Start arm control server
    arm_server = await asyncio.start_server(handle_arm_connection, 'localhost', arm_port)
    print(f"xArm Control server running on localhost:{arm_port}")

    # (4) Start physics loop
    asyncio.create_task(physics_loop())

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
    signal.signal(signal.SIGINT, signal_handler)
    try:
        asyncio.run(main())
    except:
        if running:
            traceback.print_exc()
