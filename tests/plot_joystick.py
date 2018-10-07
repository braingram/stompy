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
        v = joy.axes.get('thumb_left_x', None)
        print(v)
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
