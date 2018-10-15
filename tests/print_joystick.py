#!/usr/bin/env python

import time

import stompy


thumb_mid = 128
thumb_db = 10  # +-
thumb_scale = 127 - thumb_db

joy = stompy.joystick.ps3.PS3Joystick()

axes = {}
rel_axes = {}


def print_event(evt):
    if evt['type'] == 'axis':
        ov = axes.get(evt['code'], -100)
        if abs(evt['value'] - ov) < 25:
            return
        axes[evt['code']] = evt['value']
    if evt['ev_type'] == 0x02:
        ov = rel_axes.get(evt['code'], -100)
        if abs(evt['value'] - ov) < 25:
            return
        rel_axes[evt['code']] = evt['value']
    print(
        evt['ev_type'], evt['type'], hex(evt['code']),
        evt['name'], evt['value'])

joy.report_ev_types = set((0x01, 0x02, 0x03))
joy.on('event', print_event)
#joy.on('button', print_event)
#joy.on('axis', print_event)

while True:
    try:
        joy.update(max_time=0.05)
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
