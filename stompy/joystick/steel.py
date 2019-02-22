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
}

axis_names = {
    0: 'joy_x',
    1: 'joy_y',
    2: 'joy_z',
}

# joystick event name to stompy function mapping
# takes names defined above and maps them to names
# that the main stompy controller will understand
default_mapping = {
    'buttons': {
        'joy_button': 'deadman',
        #'one_left': 'sub_mode',
        #'left': 'leg_index_dec',
        #'right': 'leg_index_inc',
        #'up': 'speed_inc',
        #'down': 'speed_dec',
        #'select': 'mode_inc',
        #'square': 'report_stats',
        #'triangle': 'reset_stats',
    },
    'axes': {
        'joy_x': 'x',
        'joy_y': 'y',
        'joy_z': 'z',
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

    def on_axis(self, index, value):
        self._report_axis(
            axis_names.get(index.value, 'unknown'), value.value)

    def on_button(self, index, value):
        self._report_button(
            button_names.get(index.value, 'unknown'), value.value)

    def set_led(self, index, value):
        self.mgr.trigger('led', index, value)

    def update(self):
        self.com.handle_stream()
        super(SteelJoystick, self).update()
