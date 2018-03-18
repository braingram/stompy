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

lns = sorted(legs.keys())
print("Connected to legs: %s" % (lns, ))

c = stompy.controllers.multileg.MultiLeg(legs, joy)

"""
for ln in c.res.feet:
    # TODO have a way to more easily enable restriction control
    c.res.feet[ln].state = 'stance'
c.res.enable(None)
c.res.set_target((1.0, 0.0))
"""

stompy.ui.multileg.run_ui(stompy.ui.multileg.load_ui(c))
