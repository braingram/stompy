#!/usr/bin/env python3

import sys
import time

import serial

import pycomando
import stompy


if len(sys.argv) < 1:
    raise Exception("Must provide port as argument")
if len(sys.argv) < 2:
    raise Exception("Must provide leg number as second argument")
port = sys.argv[1]
leg_number = int(sys.argv[2])

comm = serial.Serial(port, 9600)
com = pycomando.Comando(comm)
cmd = pycomando.protocols.command.CommandProtocol()
com.register_protocol(0, cmd)
mgr = pycomando.protocols.command.EventManager(
    cmd, stompy.leg.teensy.cmds)
r = mgr.blocking_trigger('leg_number', leg_number)

time.sleep(0.1)
r = mgr.blocking_trigger('leg_number')[0].value
print("Leg number is now: %s" % r)
print("Don't forget to add serial number to stompy/utils.py")
