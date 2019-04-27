#!/usr/bin/env python

import time

from .. import consts
from .. import signaler


class Joystick(signaler.Signaler):
    def __init__(self, report_period=0.1):
        super(Joystick, self).__init__()
        self.report_period = 0.1
        self.last_report = time.time()
        self.buttons = {}
        self.axes = {}
        self._reset_updates()
        self.mapping = {
            'buttons': {},
            'axes': {}
        }
        self._last_deadman = time.time() - consts.HEARTBEAT_TIMEOUT - 1

    def _reset_updates(self):
        self._update = {
            'buttons': {},
            'axes': {}}

    def _check_report(self):
        t = time.time()
        if (t - self.last_report) > self.report_period:
            for k in self._update:
                if len(self._update[k]):
                    # trigger events:
                    #  'buttons', dict of buttons changed
                    #  'axes', dict of axes updated
                    self.trigger(k, self._update[k])
            self.last_report = t
            self._reset_updates()

    def _report_axis(self, axis, value):
        self._update['axes'][axis] = value
        self.axes[axis] = value
        if axis in self.mapping['axes']:
            k = self.mapping['axes'][axis]
            if isinstance(k, tuple):
                k, c = k
                value = c(value)
            self._update['axes'][k] = value
            self.axes[k] = value
        self._check_report()

    def _report_button(self, button, value):
        self._update['buttons'][button] = value
        self.buttons[button] = value
        if button == 'deadman':
            self._last_deadman = time.time()
        if button in self.mapping['buttons']:
            k = self.mapping['buttons'][button]
            self._update['buttons'][k] = value
            if k == 'deadman':
                self._last_deadman = time.time()
            self.buttons[k] = value
        self._check_report()

    def update(self):
        # if we haven't heard from the deadman, set it to 0
        if time.time() - self._last_deadman > consts.HEARTBEAT_TIMEOUT:
            self._report_button('deadman', 0)
        self._check_report()
