#!/usr/bin/env python

import logging
import sys
import time
import traceback

import pycomando
import serial

from . import log
from . import signaler
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
        'heading': 500,
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


class TeensyBody(BodyController):
    def __init__(self, port):
        self.port = port
        self.com = pycomando.Comando(serial.Serial(self.port, 9600))
        self.cmd = pycomando.protocols.command.CommandProtocol()
        self.com.register_protocol(0, self.cmd)
        mgr = pycomando.protocols.command.EventManager(
            self.cmd, base_cmds)
        self._text = pycomando.protocols.text.TextProtocol()

        name = names[mgr.blocking_trigger('name')[0].value]
        super(TeensyBody, self).__init__(name)

        self._last_hb = time.time()
        mgr.trigger('heartbeat')

        # clear callbacks, register name specific commands
        self.cmd.callbacks = {}
        self.mgr = pycomando.protocols.command.EventManager(
            self.cmd, cmds[name])

        def print_text(txt, n=name):
            print("DEBUG[body.%s]:%s" % (n, txt))

        self._text.register_callback(print_text)
        self.com.register_protocol(1, self._text)

        # setup report callbacks
        def make_callback(n):
            cbn = n

            def cb(*args):
                vs = [a.value for a in args]
                print("%s: %s" % (cbn, vs))
                if len(vs) == 1:
                    self.log.debug({cbn: vs[0]})
                else:
                    self.log.debug({cbn: vs})
                self.trigger(cbn, *vs)
            return cb

        r = reports.get(name, {})
        for k in r:
            self.mgr.on(k, make_callback(k))
            # setup reporting periods
            self.mgr.trigger(k, r[k])

    def __del__(self):
        # disable reports
        r = reports.get(self.name, {})
        print("body disabling reports")
        for k in r:
            self.mgr.trigger(k, 0)

    def update(self):
        # TODO heartbeat
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

    """
    def on(self, name, func):
        # setup when mgr calls name, trigger function
        self.mgr.on(name, func)
        # register callback with mgr
        # connect to signaler.on

    def remove_on(self, name, func):
        # opposite of on
        self.mgr.remove_on(name, func)
        # remove callback from signaler.on
        # remove callback from mgr

    def trigger(self, name, *args):
        # pass through for mgr
        self.mgr.trigger(name, *args)
    """


def connect_to_teensies(ports=None):
    if ports is None:
        tinfo = utils.find_body_teensies()
        ports = [i['port'] for i in tinfo]
    if len(ports) == 0:
        # return fake body
        return {n: BodyController(n) for n in [names[0]]}
        raise NotImplementedError
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
