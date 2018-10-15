#!/usr/bin/env python

import time

import stompy


thumb_mid = 128
thumb_db = 10  # +-
thumb_scale = 127 - thumb_db

joy = stompy.joystick.ps3.PS3Joystick()

axes = {}

def print_event(evt):
    if evt['type'] == 'axis':
        ov = axes.get(evt['code'], -100)
        if abs(evt['value'] - ov) < 25:
            return
        axes[evt['code']] = evt['value'] 
    print(evt['type'], hex(evt['code']), evt['name'], evt['value'])

joy.on('button', print_event)
joy.on('axis', print_event)

while True:
    try:
        joy.update(max_time=0.05)
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
