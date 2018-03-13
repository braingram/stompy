#!/usr/bin/env python

import stompy
import stompy.ui

if stompy.joystick.ps3.available():
    joy = stompy.joystick.ps3.PS3Joystick()
else:
    joy = None

legs = stompy.leg.teensy.connect_to_teensies()

if len(legs) == 0:
    raise IOError("No teensies found")

#leg = legs.values()[0]
#print("Connected to leg: %s" % leg.leg_name)

c = stompy.controllers.multileg.MultiLeg(legs, joy)

stompy.ui.multileg.run_ui(stompy.ui.multileg.load_ui(c))
