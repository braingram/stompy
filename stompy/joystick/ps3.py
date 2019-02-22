#!/usr/bin/env python

import os
import select
import struct
import threading
import time

from . import base
from .. import signaler


DEFAULT_FN = '/dev/input/by-id/usb-Sony_' \
    'PLAYSTATION_R_3_Controller-event-joystick'
#DEFAULT_FN = '/dev/input/by-id/usb-SHANWAN_' \
#    'PS3_GamePad-joystick'

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

codes_by_version = {
    '14.04': {
        'keys': dict([  # 0x01, 17
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
        ]),
        'misc': dict([]),  # 0x04

        'abs_axes': dict([  # 0x03, value maxes at 255
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
        ]),
    },
    '18.04': {
        'keys': dict([
            (0x13a, 'select'),
            (0x13d, 'left_thumb'),
            (0x13e, 'right_thumb'),
            (0x13b, 'start'),
            (0x220, 'up'),
            (0x223, 'right'),
            (0x221, 'down'),
            (0x222, 'left'),
            (0x138, 'two_left'),
            (0x139, 'two_right'),
            (0x136, 'one_left'),
            (0x137, 'one_right'),
            (0x133, 'triangle'),
            (0x131, 'circle'),
            (0x130, 'cross'),
            (0x134, 'square'),
            (0x13c, 'ps'),
        ]),
        'misc': dict([]),  # 0x04
        'abs_axes': dict([  # 0x03, value maxes at 255
            # I have no way to test
            (0x00, 'thumb_left_x'),  # left = decrease
            (0x01, 'thumb_left_y'),  # up = decrease
            (0x03, 'thumb_right_x'),  # left = decrease
            (0x04, 'thumb_right_y'),  # up = decrease
            # left?
            #(0x2c, 'up'),
            #(0x2d, 'right'),
            #(0x2e, 'down'),
            (0x02, 'two_left'),
            (0x05, 'two_right'),
            #(0x32, 'one_left'),
            #(0x33, 'one_right'),
            #(0x34, 'triangle'),
            #(0x35, 'circle'),
            #(0x36, 'cross'),
            #(0x37, 'square'),
            #(0x3b, 'acc_x'),  # left/right
            #(0x3c, 'acc_y'),  # forward/back
            #(0x3d, 'acc_z'),  # up/down
        ]),

    },
}

default_mapping = {
    'buttons': {
        'one_right': 'deadman',
        'one_left': 'sub_mode',
        'square': 'report_stats',
        'triangle': 'reset_stats',
        'left': 'leg_index_dec',
        'right': 'leg_index_inc',
        'up': 'speed_inc',
        'down': 'speed_dec',
        'select': 'mode_inc',
    },
    'axes': {
        'x0': 'thumb_left_x',
        'y0': 'thumb_left_y',
        'x1': 'thumb_right_x',
        'y1': 'thumb_right_y',
    },
}

POLL_TIMEOUT = 0.001
THREAD_SLEEP = 0.01


def available(fn=None):
    if fn is None:
        fn = DEFAULT_FN
    return os.path.exists(fn)


#class PS3Joystick(signaler.Signaler):
class PS3Joystick(base.Joystick):
    def __init__(self, fn=None):
        super(PS3Joystick, self).__init__()
        self.mapping = default_mapping
        self.codes = None
        if fn is None:
            fn = DEFAULT_FN
        self.fn = fn
        self.f = open(fn, 'rb+')
        #self.keys = {}
        #self.axes = {}
        #self.report_ev_types = set((0x01, 0x03))

    def lookup_name(self, code, key, default='unknown'):
        if self.codes is None:
            # check against possible codes
            if key != 'keys':  # only do this for keys
                return default
            for v in codes_by_version:
                if code in codes_by_version[v]['keys']:
                    self._version = v
                    self.codes = codes_by_version[v]
                    break
            else:
                return default
        if key not in self.codes:
            return default
        return self.codes[key].get(code, default)

    def read_event(self):
        t_sec, t_usec, ev_type, code, value = struct.unpack(
            FMT, self.f.read(NB))

        e = {
            'ev_type': ev_type,
            'code': code,
            'time': t_sec + t_usec / 1000000.,
            'value': value}
        if ev_type == 0x01:  # keys
            e['type'] = 'button'
            e['name'] = self.lookup_name(code, 'keys')
            self._report_button(e['name'], value)
            #self.keys[e['name']] = value
        elif ev_type == 0x03:  # axes
            e['type'] = 'axis'
            e['name'] = self.lookup_name(code, 'abs_axes')
            self._report_axis(e['name'], value)
            #self.axes[e['name']] = value
        return e

    def update(self, max_time=0.005):
        # read multiple events per update
        st = time.time()
        evs = []
        while (time.time() - st) < max_time:
            rf, _, _ = select.select([self.f, ], [], [], POLL_TIMEOUT)
            if len(rf) == 0:
                break
            e = self.read_event()
            #if e['ev_type'] in self.report_ev_types:
            #    evs.append(e)
            #    self.trigger('event', e)
            #    if 'type' in e:
            #        self.trigger(e['type'], e)
        super(PS3Joystick, self).update()
        return evs

    def _update_thread_function(self):
        while True:
            self.update(poll=True)
            time.sleep(THREAD_SLEEP)

    def start_update_thread(self):
        self._update_thread = threading.Thread(
            target=self._update_thread_function)
        self._update_thread.daemon = True
        self._update_thread.start()


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
