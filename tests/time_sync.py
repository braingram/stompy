#!/usr/bin/env python

import ctypes

import time

import pycomando
import serial


global receipt_time, teensy_time
receipt_time = None
teensy_time = None


def new_time(cmd):
    global receipt_time, teensy_time
    receipt_time = time.time()
    teensy_time = cmd.get_arg(ctypes.c_uint).value / 1000.


def get_time():
    global teensy_time, receipt_time
    teensy_time = None
    receipt_time = None
    sent_time = time.time()
    cmd.send_command(5)
    while teensy_time is None:
        com.handle_stream()
    return teensy_time, sent_time, receipt_time


# connect to teensy
conn = serial.Serial('/dev/ttyACM0', 115200)
com = pycomando.Comando(conn)
text = pycomando.protocols.TextProtocol(com)
cmd = pycomando.protocols.CommandProtocol(com)
com.register_protocol(0, text)
com.register_protocol(1, cmd)

cmd.register_callback(5, new_time)

# sync
t1p, t1, _ = get_time()
# delay_request
t2, _, t2p = get_time()
# compute offset
offset = (t1p - t1 - t2p + t2) / 2.
print("Times:")
print("\tt1p: %s" % t1p)
print("\tt1 : %s" % t1)
print("\tt2p: %s" % t2p)
print("\tt2 : %s" % t2)
print("Clock offset: %s" % offset)

# test_offset
tt, st, rt = get_time()
serr = (st + offset) - tt
print("Error (using sent time): %s" % serr)
rerr = (rt + offset) - tt
print("Error (using receipt time): %s" % rerr)
