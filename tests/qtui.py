#!/usr/bin/env python

import stompy
import stompy.ui

if not stompy.joystick.ps3.available():
    raise IOError("Requires joystick, none found")

joy = stompy.joystick.ps3.PS3Joystick()
legs = stompy.leg.teensy.connect_to_teensies()

if len(legs) == 0:
    raise IOError("No teensies found")

if len(legs) > 1:
    raise NotImplementedError("leg selection not implemented")

leg = legs.values()[0]
print("Connected to leg: %s" % leg.leg_name)

c = stompy.controllers.single_leg.SingleLeg(leg, joy)

stompy.ui.ui.run_ui(stompy.ui.ui.load_ui(c))
