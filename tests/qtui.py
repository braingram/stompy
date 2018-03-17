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
    x, y, = legs[i].res.center
    #z = legs[i].res.lift_height
    z = 0
    legs[i].send_plan(
        frame=stompy.consts.PLAN_LEG_FRAME,
        mode=stompy.consts.PLAN_TARGET_MODE,
        linear=(x, y, z),
        speed=5.)
for ln in legs:
    legs[ln].res.set_target(-1, 0, stompy.consts.PLAN_BODY_FRAME)
    legs[ln].send_plan(**legs[ln].res.plans['stance'])
    legs[ln].res.set_state('stance')

stompy.ui.multileg.run_ui(stompy.ui.multileg.load_ui(c))
