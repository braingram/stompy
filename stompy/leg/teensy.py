#!/usr/bin/env python

import ctypes
import time
import threading

import serial

import pycomando


commands = {
    0: {
        'name': 'joints',
        'result': (
            ctypes.c_uint, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float),
    },
    1: {
        'name': 'estop',
        'args': (ctypes.c_byte, ),
        'result': (ctypes.c_byte, ),
    },
    2: {
        'name': 'enable',
        'args': (ctypes.c_bool, ),
    },
    3: {
        'name': 'heartbeat',
    },
    4: {
        'name': 'status',
        'result': (ctypes.c_bool, ),
    },
}

global conn, com, text, cmd, mgr, lock
conn = None
com = None
text = None
cmd = None
mgr = None
lock = threading.Lock()


def connect(port='/dev/ttyACM0', baud=115200):
    global conn, com, mgr
    conn = serial.Serial(port, baud)
    com = pycomando.Comando(conn)
    text = pycomando.protocols.TextProtocol(com)
    cmd = pycomando.protocols.CommandProtocol(com)
    com.register_protocol(0, text)
    com.register_protocol(1, cmd)
    mgr = pycomando.protocols.command.EventManager(cmd, commands)
    return mgr


def test_dropped_sbc_heartbeat():

    def make_printer(header):
        def printer(*args):
            args = [getattr(a, 'value', a) for a in args]
            print("%s: %s" % (header, args))
        return printer

    global last_beat
    last_beat = time.time()

    def on_beat():
        global last_beat
        last_beat = time.time()
        print("heartbeat: [%s]" % last_beat)

    # connect
    if mgr is None:
        connect()

    # connect callbacks
    mgr.on('joints', make_printer('joints'))
    mgr.on('estop', make_printer('estop'))
    mgr.on('enable', make_printer('enable'))
    #mgr.on('heartbeat', make_printer('heartbeat'))
    mgr.on('heartbeat', on_beat)

    last_sent_beat = time.time()
    mgr.trigger('heartbeat')

    beats = 0
    while True:
        try:
            t = time.time()
            if t - last_sent_beat > 0.5 and beats < 5:
                last_sent_beat = time.time()
                print("sending heartbeat")
                mgr.trigger('heartbeat')
                beats += 1
            com.handle_stream()
            if t - last_beat > 1.0:
                print("!! heartbeat lost !!")
                print("sending estop")
                mgr.trigger('estop', 0)
                break
        except KeyboardInterrupt as e:
            break


def test():
    test_dropped_sbc_heartbeat()
