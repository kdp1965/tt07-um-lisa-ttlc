#!/usr/bin/env python3
"""TCP <-> serial bridge: lets hw_test.mjs drive the board through a Web-Serial-like shim.

    python3 bridge.py [serial port] [tcp port]      (defaults: /dev/cu.usbmodem11101, 5555)
"""
import socket
import sys
import threading
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/cu.usbmodem11101'
TCP = int(sys.argv[2]) if len(sys.argv) > 2 else 5555

ser = serial.Serial(PORT, 115200, timeout=0.001)
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('127.0.0.1', TCP))
srv.listen(1)
print('bridge ready on', TCP, flush=True)
while True:
    conn, _ = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    alive = [True]

    def s2t():
        while alive[0]:
            d = ser.read(4096)
            if d:
                try:
                    conn.sendall(d)
                except OSError:
                    break

    t = threading.Thread(target=s2t, daemon=True)
    t.start()
    try:
        while True:
            try:
                d = conn.recv(4096)
            except OSError:
                break
            if not d:
                break
            ser.write(d)
    finally:
        alive[0] = False
        conn.close()
        t.join()
