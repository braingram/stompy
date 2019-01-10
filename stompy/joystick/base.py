#!/usr/bin/env python

import time

from .. import signaler


class Joystick(signaler.Signaler):
    def __init__(self, report_period=0.1):
        super(Joystick, self).__init__()
        self.report_period = 0.1
        self.last_report = time.time()
        self.buttons = {}
        self.axes = {}
        self._reset_updates()

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
        self._check_report()

    def _report_button(self, button, value):
        self._update['buttons'][button] = value
        self.buttons[button] = value
        self._check_report()

    def update(self):
        self._check_report()
