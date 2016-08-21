#!/usr/bin/env python

import ctypes

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
        'args': (ctypes.c_byte),
    },
    2: {
        'name': 'enable',
        'args': (ctypes.c_bool),
    },
    3: {
        'name': 'heartbeat',
    },
}

global mgr
mgr = None


def connect(port='/dev/ttyUSB0', baud=115200):
    conn = serial.Serial(port, baud)
    com = pycomando.Comando(conn)
    text = pycomando.protocols.TextProtocol(com)
    cmd = pycomando.protocols.CommandProtocol(com)
    com.register_protocol(0, text)
    com.register_protocol(1, cmd)
    global mgr
    mgr = pycomando.protocols.command.EventManager(cmd, commands)
    return mgr
