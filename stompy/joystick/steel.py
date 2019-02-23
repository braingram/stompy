#!/usr/bin/env python

import time

import pycomando
import serial

from . import base
from .. import utils


cmds = {
    0: 'reset',
    1: 'axis=byte,uint8',
    2: 'button=byte,uint8',
    3: 'led(byte,byte)',
}

button_names = {
    0: 'joy_button',
    1: 'top_right_switch',
    2: 'top_middle_switch',
    3: 'top_left_switch',
    4: 'upper_left_button',
    5: 'middle_left_button',
    6: 'lower_left_button',
    7: 'leg_1',
    8: 'leg_2',
    9: 'leg_3',
    10: 'leg_4',
    11: 'leg_5',
    12: 'leg_6',
}

axis_names = {
    0: 'joy_x',
    1: 'joy_y',
    2: 'joy_z',
    3: 'left_slider',
    4: 'right_slider',
    5: 'top_knob',
    6: 'bottom_knob',
}

# joystick event name to stompy function mapping
# takes names defined above and maps them to names
# that the main stompy controller will understand
default_mapping = {
    'buttons': {
        'joy_button': 'deadman',
        'top_right_switch': 'use_sliders_switch',
        'top_middle_switch': 'body_walk_switch',
        'top_left_switch': 'leg_mode_switch',
        #'upper_left_button': ,
        'middle_left_button': 'sub_mode',
        'lower_left_button': 'restrict_leg',
        'leg_1': 'leg_select_1',
        'leg_2': 'leg_select_2',
        'leg_3': 'leg_select_3',
        'leg_4': 'leg_select_4',
        'leg_5': 'leg_select_5',
        'leg_6': 'leg_select_6',
    },
    'axes': {
        'joy_x': 'x',
        'joy_y': 'y',
        'joy_z': 'z',
        'left_slider': 'speed_axis',
        'right_slider': 'height_axis',
    },
}


def available():
    ts = utils.find_teensies_by_type('joystick')
    return len(ts)


class SteelJoystick(base.Joystick):
    def __init__(self, port=None):
        super(SteelJoystick, self).__init__()
        self.mapping = default_mapping
        if port is None:
            ts = utils.find_teensies_by_type('joystick')
            if len(ts) != 1:
                raise IOError("Didn't find custom joystick: %s" % (ts, ))
            port = ts[0]['port']
        self.port = port
        self.com = pycomando.Comando(serial.Serial(self.port, 9600))
        self.cmd = pycomando.protocols.command.CommandProtocol()
        self.com.register_protocol(0, self.cmd)
        #self.text = pycomando.protocols.TextProtocol()
        self.mgr = pycomando.protocols.command.EventManager(self.cmd, cmds)
        # attach callbacks
        self.mgr.on('axis', self.on_axis)
        self.mgr.on('button', self.on_button)
        self.mgr.trigger('reset')

    def on_axis(self, index, value):
        #print(time.time(), axis_names[index.value], value.value)
        self._report_axis(
            axis_names.get(index.value, 'unknown'), value.value)

    def on_button(self, index, value):
        #print(time.time(), button_names[index.value], value.value)
        self._report_button(
            button_names.get(index.value, 'unknown'), value.value)

    def set_led(self, index, value):
        self.mgr.trigger('led', index, value)

    def update(self):
        self.com.handle_stream()
        super(SteelJoystick, self).update()
