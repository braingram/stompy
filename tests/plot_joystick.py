#!/usr/bin/env python

import time

import stompy


thumb_mid = 128
thumb_db = 10  # +-
thumb_scale = 127 - thumb_db

joy = stompy.joystick.ps3.PS3Joystick()

while True:
    try:
        joy.update(max_time=0.05)
        a = joy.smoothed_axes.get('thumb_left_x', None)
        if a is None:
            v = thumb_mid
        else:
            v = a.update()
        print(v)
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
