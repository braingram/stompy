#!/usr/bin/env python

import logging
import sys
import time
import traceback

import pycomando
import serial

from . import log
from . import signaler
from . import simulation
from . import utils


logger = logging.getLogger(__name__)

names = {
    0: 'imu',
}

base_cmds = {
    0: 'name=byte',
    1: 'heartbeat',
}

cmds = {
    'imu': {
        0: 'name=byte',
        1: 'heartbeat',
        # (poll frequency [ms])=pressure [psi]
        2: 'feed_pressure(byte)=float',
        3: 'return_pressure(byte)=float',

        # (poll freq [ms])=temp [F]
        5: 'feed_oil_temp(byte)=float',
        6: 'return_oil_temp(byte)=float',
        7: 'engine_temp(byte)=float',

        # (poll freq [ms])=volts
        10: 'battery_voltage(byte)=float',
        # (poll freq [ms])=amps
        11: 'battery_current(byte)=float',

        # (poll freq [ms])=roll,pitch,yaw [degrees]
        15: 'heading(byte)=float,float,float',

        # (poll freq [ms])=tank level %
        20: 'propane(byte)=float',
        # (poll freq [ms])=rpm
        21: 'engine_rpm(byte)=float',
        # (poll freq [ms])=temp [F]
        22: 'engine_oil_temp(byte)=float',

        # -- actuators --
        # throttle % [or none for get]=throttle %
        30: 'throttle(float)=float',
        # 1(on)0(off) [or none for get]=same
        31: 'big_pump(byte)=byte',
        32: 'little_pump(byte)=byte',
        33: 'fans(byte)=byte',
    },
}

reports = {
    'imu': {
        'feed_pressure': 100,  # ms
        'feed_oil_temp': 500,
        #'heading': 500,
        'engine_rpm': 500,
    },
}


class BodyController(signaler.Signaler):
    def __init__(self, name):
        super(BodyController, self).__init__()
        self.name = name
        self.log = log.make_logger(self.name)

    def update(self):
        # fake any sensors here
        pass


class FakeIMU(BodyController):
    def __init__(self):
        super(FakeIMU, self).__init__('imu')
        # TODO connect to simulation
        self._last_report = time.time()
        self._sim = simulation.get()

    def update(self):
        # emit:
        t = time.time()
        if (t - self._last_report) >= 0.1:
            # - feed_pressure
            self.trigger('feed_pressure', 1700)
            # - engine_rpm
            self.trigger('engine_rpm', 1500)
            # - heading [roll, pitch, yaw]
            # TODO confirm directions
            ori = self._sim.get_orientation()
            self.trigger('heading', *ori)
            self._last_report = t


class TeensyBody(BodyController):
    def __init__(self, port):
        self.port = port
        self._serial = serial.Serial(self.port, 9600)
        # set rising edge of RTS to reset comando
        self._serial.setRTS(0)
        self._serial.flushInput()
        self._serial.flushOutput()
        self._serial.setRTS(1)
        time.sleep(0.1)
        self.com = pycomando.Comando(self._serial)
        self.cmd = pycomando.protocols.command.CommandProtocol()
        self.com.register_protocol(0, self.cmd)
        mgr = pycomando.protocols.command.EventManager(
            self.cmd, base_cmds)
        self._text = pycomando.protocols.text.TextProtocol()

        self.name = names[mgr.blocking_trigger('name')[0].value]
        super(TeensyBody, self).__init__(self.name)

        self._last_hb = time.time()
        mgr.trigger('heartbeat')

        # clear callbacks, register name specific commands
        self.cmd.callbacks = {}
        self.mgr = pycomando.protocols.command.EventManager(
            self.cmd, cmds[self.name])

        def print_text(txt, n=self.name):
            print("DEBUG[body.%s]:%s" % (n, txt))

        self._text.register_callback(print_text)
        self.com.register_protocol(1, self._text)

        # setup report callbacks
        def make_callback(n):
            cbn = n

            def cb(*args):
                vs = [a.value for a in args]
                #print("%s: %s" % (cbn, vs))
                if len(vs) == 1:
                    self.log.debug({cbn: vs[0]})
                else:
                    self.log.debug({cbn: vs})
                self.trigger(cbn, *vs)
            return cb

        r = reports.get(self.name, {})
        for k in r:
            self.mgr.on(k, make_callback(k))
            # setup reporting periods
            self.mgr.trigger(k, r[k])

        self.mgr.on('heading', self._on_heading)
        self.mgr.trigger('heading', 500)

    def _on_heading(self, roll, pitch, yaw):
        # roll + is to the right
        # pitch + is up
        r, p, y = (roll.value, pitch.value, yaw.value)
        # correct for offsets
        r += 3.3375 # 2.4875
        p += 4.3  # 2.6375
        self.log.debug({'heading': (r, p, y)})
        self.trigger('heading', r, p, y)

    def __del__(self):
        if not hasattr(self, 'name'):
            return
        # disable reports
        r = reports.get(self.name, {})
        #print("body disabling reports")
        for k in r:
            self.mgr.trigger(k, 0)

    def update(self):
        # heartbeat
        t = time.time()
        if (t - self._last_hb > 0.5):
            self.mgr.trigger('heartbeat')
            self._last_hb = t
        try:
            self.com.handle_stream()
        except Exception as e:
            ex_type, ex, tb = sys.exc_info()
            tbs = '\n'.join(traceback.format_tb(tb))
            self.log.error("handle_stream error: %s" % e)
            self.log.error({'error': {
                'traceback': tbs,
                'exception': e}})
            raise e


def connect_to_teensies(ports=None):
    if ports is None:
        #tinfo = utils.find_body_teensies()
        #ports = [i['port'] for i in tinfo]
        ports = [t['port'] for t in utils.find_teensies_by_type('body')]
        if any([p is None for p in ports]):
            print(
                'Body teensies: %s' % (utils.find_teensies_by_type('body'), ))
            raise IOError("Failed to find port for a body teensy")
    if len(ports) == 0:
        # return fake body
        #return {n: BodyController(n) for n in [names[0]]}
        return {}
        #raise NotImplementedError
    teensies = [TeensyBody(p) for p in ports]
    nd = {}
    for t in teensies:
        n = t.name
        nd[n] = nd.get(n, []) + [t, ]
    for n in nd:
        if len(nd[n]) > 1:
            raise Exception(
                "Found >1 body teensies with the same name: %s[%s]" %
                (n, nd[n]))
        nd[n] = nd[n][0]
    return nd
