#!/usr/bin/env python

import time

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

# send initial target
for i in legs:
    legs[i].res.set_target(-1, 0, stompy.consts.PLAN_BODY_FRAME)
    legs[i].send_plan(**legs[i].res.plans['swing'])
    # update leg until close to target
for i in xrange(10):
    for ln in legs:
        legs[ln].update()
    time.sleep(0.101)
for ln in legs:
    legs[ln].res.set_state('swing')

stompy.ui.multileg.run_ui(stompy.ui.multileg.load_ui(c))
