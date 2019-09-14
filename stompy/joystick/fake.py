#!/usr/bin/env python

from . import base


default_mapping = {
    'buttons': {
        'one_right': 'deadman',
        'one_left': 'sub_mode',
        'square': 'report_stats',
        'cross': 'restrict_leg',
        'triangle': 'reset_stats',
        'left': 'leg_index_dec',
        'right': 'leg_index_inc',
        'up': 'speed_inc',
        'down': 'speed_dec',
        'select': 'mode_inc',
    },
    'axes': {
        'thumb_left_x': 'x',
        'thumb_left_y': ('y', lambda v: 255 - v),
        'thumb_right_y': ('z', lambda v: 255 - v),
    },
}


class FakeJoystick(base.Joystick):
    def __init__(self):
        super(FakeJoystick, self).__init__()
        # add mapping in subclass

    def set_button(self, name, value):
        self._report_button(name, value)

    def set_axis(self, name, value):
        self._report_axes(name, value)

    #def build_ui(self):
    #    if hasattr(self, '_ui'):
    #        return
    #    self._ui = FakeJoystickUI()
    #    self._ui.resize(200, 200)
    #    self._ui.show()
