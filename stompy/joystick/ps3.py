#!/usr/bin/env python

import os
import select
import struct
import threading
import time


DEFAULT_FN = '/dev/input/by-id/usb-Sony_' \
    'PLAYSTATION_R_3_Controller-event-joystick'

FMT = 'llHHi'
NB = struct.calcsize(FMT)
# 0, 1, 3, 4
EVS = dict([
    (0x00, "Sync"),
    (0x01, "Key"),
    (0x02, "Relative"),
    (0x03, "Absolute"),
    (0x04, "Misc"),
    (0x05, "Switch"),
    (0x11, "LED"),
    (0x12, "Sound"),
    (0x14, "Repeat"),
    (0x15, "ForceFeedback"),
    (0x16, "Power"),
    (0x17, "ForceFeedbackStatus"),
    (0x1f, "Max"),
    (0x1f+1, "Current")])

KEYS = dict([  # 0x01, 17
    (0x120, 'select'),
    (0x121, 'left_thumb'),
    (0x122, 'right_thumb'),
    (0x123, 'start'),
    (0x124, 'up'),
    (0x125, 'right'),
    (0x126, 'down'),
    (0x127, 'left'),
    (0x128, 'two_left'),
    (0x129, 'two_right'),
    (0x12a, 'one_left'),
    (0x12b, 'one_right'),
    (0x12c, 'triangle'),
    (0x12d, 'circle'),
    (0x12e, 'cross'),
    (0x12f, 'square'),
    (0x2c0, 'ps'),
])

MISC = dict([  # 0x04
])

ABS_AXES = dict([  # 0x03, value maxes at 255
    (0x00, 'thumb_left_x'),  # left = decrease
    (0x01, 'thumb_left_y'),  # up = decrease
    (0x02, 'thumb_right_x'),  # left = decrease
    (0x05, 'thumb_right_y'),  # up = decrease
    # left?
    (0x2c, 'up'),
    (0x2d, 'right'),
    (0x2e, 'down'),
    (0x30, 'two_left'),
    (0x31, 'two_right'),
    (0x32, 'one_left'),
    (0x33, 'one_right'),
    (0x34, 'triangle'),
    (0x35, 'circle'),
    (0x36, 'cross'),
    (0x37, 'square'),
    (0x3b, 'acc_x'),  # left/right
    (0x3c, 'acc_y'),  # forward/back
    (0x3d, 'acc_z'),  # up/down
])

POLL_TIMEOUT = 0.001
THREAD_SLEEP = 0.01


def available(fn=None):
    if fn is None:
        fn = DEFAULT_FN
    return os.path.exists(fn)


class PS3Joystick(object):
    def __init__(self, fn=None):
        if fn is None:
            fn = DEFAULT_FN
        self.fn = fn
        self.f = open(fn, 'rb+')
        self.keys = {}
        self.key_edges = {}
        self.axes = {}

    def clear_key_edge(self, name):
        if name in self.key_edges:
            del self.key_edges[name]

    def update(self, poll=True):
        if poll:
            rf, _, _ = select.select([self.f, ], [], [], POLL_TIMEOUT)
            if len(rf) == 0:
                return None
        t_sec, t_usec, ev_type, code, value = struct.unpack(
            FMT, self.f.read(NB))

        e = {
            'ev_type': ev_type,
            'code': code,
            'time': t_sec + t_usec / 1000000.,
            'value': value}
        if ev_type == 0x01:  # keys
            e['type'] = 'button'
            e['name'] = KEYS.get(code, 'unknown')
            old_value = self.keys.get(e['name'], None)
            if old_value is None or old_value != value:
                self.key_edges[e['name']] = e
            self.keys[e['name']] = value
        elif ev_type == 0x03:  # axes
            e['type'] = 'axis'
            e['name'] = ABS_AXES.get(code, 'unknown')
            self.axes[e['name']] = value
        return e

    def _update_thread_function(self):
        while True:
            ev = self.update(poll=True)
            while ev is not None:
                ev = self.update(poll=True)
            time.sleep(THREAD_SLEEP)

    def start_update_thread(self):
        self._update_thread = threading.Thread(
            target=self._update_thread_function)
        self._update_thread.daemon = True
        self._update_thread.start()

    def write_event(self, ev_type, code, value):
        return
        t = time.time()
        t_sec = int(t)
        t_usec = int(t - t_sec * 1000000)
        msg = struct.pack(FMT, t_sec, t_usec, ev_type, code, value)
        self.f.write(msg)


def test_read_axes():
    ignore = {
        0x00: True,
        0x01: True,
        0x03: True,
        #0x03: {
        #    0x3b: True,
        #    0x3c: True,
        #    0x3d: True,
        #    0x00: True,
        #    0x01: True,
        #    0x02: True,
        #    0x05: True,
        #},
        #0x04: True,
    }

    axes = {}

    with open(DEFAULT_FN, 'rb') as f:
        while True:
            try:
                # rf, _, _ = select.select([f, ], [], [], 0.001)
                # if len(rf):
                #     pass
                t_sec, t_usec, ev_type, code, value = struct.unpack(
                    FMT, f.read(NB))
                if ev_type in ignore:
                    if ignore[ev_type] is True:
                        continue
                    if code in ignore[ev_type]:
                        continue
                print(hex(ev_type), hex(code), value)
                if ev_type == 0x03:
                    if code not in axes:
                        axes[code] = []
                    axes[code].append([t_sec + t_usec / 1000000., value])
            except KeyboardInterrupt:
                break

    #if len(axes):
    #    for ax in axes:
    #        a = numpy.array(axes[ax])
    #        pylab.plot(a[:, 0], a[:, 1], label=str(ax))
    #    pylab.legend()
    #    pylab.show()
    return axes
