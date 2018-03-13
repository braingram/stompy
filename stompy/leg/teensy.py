#!/usr/bin/env python

import glob
import logging
import subprocess
import threading
import time

import numpy
import serial

import pycomando

from .. import consts
from .. import calibration
from . import plans
from .. import log


logger = logging.getLogger(__name__)

cmds = {
    0: 'heartbeat',
    1: 'estop(byte)=byte',  # 0 = off, 1 = soft, 2 = hard
    2: 'pwm(float,float,float)=float,float,float',  # hip thigh knee
    3: 'plan(byte,byte,float,float,float,float,float,float,float)='
       'byte,byte,float,float,float,float,float,float,float',
    4: 'enable_pid(bool)=bool',
    5: 'pid_config(byte,float,float,float,float,float)='
       'byte,float,float,float,float,float',
    6: 'leg_number(byte)=byte',
    7: 'pwm_limits(byte,int32,int32,int32,int32)=byte,int32,int32,int32,int32',
    8: 'adc_limits(byte,float,float)=byte,float,float',
    9: 'calf_scale(float,float)=float,float',
    10: 'report_time(uint32)=uint32',
    11: 'pid_seed_time(uint32,uint32)=uint32,uint32',
    12: 'reset_pids(bool)',  # i_only
    13: 'dither(byte,uint32,int)=byte,uint32,int',

    21: 'report_adc(bool)=uint32,uint32,uint32,uint32',
    22: 'report_pid(bool)='
        'float,float,float,float,float,float,float,float,float',
    23: 'report_pwm(bool)=int32,int32,int32',
    24: 'report_xyz(bool)=float,float,float',
    25: 'report_angles(bool)=float,float,float,float,bool',
    26: 'report_loop_time(bool)=uint32',
}


def usb_serial_port_info(port_path=None, glob_string='/dev/ttyACM*'):
    if port_path is None:
        ports = glob.glob(glob_string)
        return [usb_serial_port_info(p) for p in ports]
    dev_path = subprocess.check_output(
        ("udevadm info -q path -n %s" % port_path).split()).strip()
    info = subprocess.check_output(
        ("udevadm info -p %s" % dev_path).split()).strip()
    d = {'port': port_path, 'dev_path': dev_path}
    for l in info.split('\n'):
        t = l.split()
        if len(t) > 1 and '=' in t[1]:
            st = t[1].split('=')
            if len(st) == 2:
                d[st[0]] = st[1]
    return d


def find_teensies():
    info = usb_serial_port_info()
    tinfo = []
    for i in info:
        if i['ID_VENDOR'] != 'Teensyduino':
            continue
        tinfo.append({
            'port': i['port'],
            'serial': i['ID_SERIAL_SHORT']})
    return tinfo


THREAD_SLEEP = 0.01


class Teensy(object):
    foot_travel_center = (66.0, 0.0)
    foot_travel_radius = 42.0
    foot_r_eps = numpy.log(0.1) / foot_travel_radius
    dr_smooth = 0.5  # should be 0 -> 0.9999: higher = more dr smoothing

    def __init__(self, port):
        self.port = port
        self.com = pycomando.Comando(serial.Serial(self.port, 9600))
        self.cmd = pycomando.protocols.command.CommandProtocol()
        self.com.register_protocol(0, self.cmd)
        # used for callbacks
        self.mgr = pycomando.protocols.command.EventManager(self.cmd, cmds)
        # easier for calling
        # self.ns = self.mgr.build_namespace()
        # get leg number
        logger.debug("%s Get leg number" % port)
        self.leg_number = self.mgr.blocking_trigger('leg_number')[0].value
        log.info({'leg_number': self.leg_number})
        logger.debug("%s leg number = %s" % (port, self.leg_number))

        self.leg_name = consts.LEG_NAME_BY_NUMBER[self.leg_number]
        log.info({'leg_name': self.leg_name})

        # load calibration setup
        for v in calibration.setup.get(self.leg_number, []):
            log.debug({'calibration': v})
            f, args = v
            logger.debug("Calibration: %s, %s" % (f, args))
            self.mgr.trigger(f, *args)

        # disable leg
        self.set_estop(consts.ESTOP_DEFAULT)
        # send first heartbeat
        self.send_heartbeat()

        # state
        # -- status
        #   - estop [state from python/firmware?]
        #   - heartbeat [time from python]
        #   - enable_pid [state from python/firmware?]
        # -- low level
        #   - adc [hip, thigh, knee, calf] {uint32}
        #   - pwm_value [hip, thigh, knee] {int32}
        #   - pid [h_output, to, ko, h_set, ts, ks, h_err, te, ke]
        # -- high level
        #   - xyz {float}
        self.xyz = {}
        #   - angles [hip, thigh, knee, calf, valid] {float,...,byte}
        self.angles = {}
        self.pid = {}
        self.pwm_value = {}

        self.mgr.on('report_xyz', self.on_report_xyz)
        self.mgr.on('report_angles', self.on_report_angles)
        self.mgr.on('report_pid', self.on_report_pid)
        self.mgr.on('report_pwm', self.on_report_pwm)
        self.mgr.on('report_adc', self.on_report_adc)

    def set_estop(self, value):
        self.mgr.trigger('estop', value)
        log.info({'estop': value})

    def enable_pid(self, value):
        self.mgr.trigger('enable_pid', value)
        log.debug({'enable_pid': value})

    def on_report_adc(self, hip, thigh, knee, calf):
        self.adc = {
            'hip': hip.value, 'thigh': thigh.value,
            'knee': knee.value, 'calf': calf.value,
            'time': time.time()}
        log.debug({'adc': self.adc})

    def restriction(self, x, y, z):
        cx, cy = self.foot_travel_center
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        return numpy.exp(-self.foot_r_eps * (d - self.foot_travel_radius))

    def on_report_xyz(self, x, y, z):
        t = time.time()
        x, y, z = x.value, y.value, z.value
        self.r = self.restriction(x, y, z)
        # compute smooted dr
        if 'r' not in self.xyz or 'time' not in self.xyz:
            self.dr = 0.
            idr = 0.
        else:
            idr = (self.r - self.xyz['r']) / (t - self.xyz['time'])
            self.dr = self.dr * self.dr_smooth + idr * (1. - self.dr_smooth)
        self.xyz = {
            'x': x, 'y': y, 'z': z,
            'r': self.r, 'dr': self.dr, 'idr': idr,
            'time': t}
        self.xyz['dr'] = self.dr
        log.debug({'xyz': self.xyz})

    def on_report_angles(self, h, t, k, c, v):
        self.angles = {
            'hip': h.value, 'thigh': t.value, 'knee': k.value,
            'calf': c.value,
            'valid': bool(v), 'time': time.time()}
        log.debug({'angles': self.angles})

    def on_report_pid(self, ho, to, ko, hs, ts, ks, he, te, ke):
        self.pid = {
            'time': time.time(),
            'output': {
                'hip': ho.value,
                'thigh': to.value,
                'knee': ko.value,
            },
            'set_point': {
                'hip': hs.value,
                'thigh': ts.value,
                'knee': ks.value,
            },
            'error': {
                'hip': he.value,
                'thigh': te.value,
                'knee': ke.value,
            }}
        log.debug({'pid': self.pid})

    def on_report_pwm(self, h, t, k):
        self.pwm = {
            'hip': h.value, 'thigh': t.value, 'knee': k.value,
            'time': time.time()}
        log.debug({'pwm': self.pwm})

    def send_heartbeat(self):
        self.mgr.trigger('heartbeat')
        self.last_heartbeat = time.time()
        # print("HB: %s" % self.last_heartbeat)

    def update(self):
        self.com.handle_stream()
        if time.time() - self.last_heartbeat > consts.HEARTBEAT_PERIOD:
            self.send_heartbeat()

    def _update_thread_function(self):
        while True:
            self.update()
            time.sleep(THREAD_SLEEP)

    def start_update_thread(self):
        self._update_thread = threading.Thread(
            target=self._update_thread_function)
        self._update_thread.daemon = True
        self._update_thread.start()

    def send_plan(self, *args, **kwargs):
        if len(args) == 0:
            return self.stop()
        if len(args) == 1 and isinstance(args[0], plans.Plan):
            plan = args[0]
        else:
            plan = plans.Plan(*args, **kwargs)
        pp = plan.packed(self.leg_number)
        # print("sending: %s" % (pp, ))
        log.info({'plan': pp})
        self.mgr.trigger('plan', *pp)

    def stop(self):
        """Send stop plan"""
        self.send_plan(plans.stop())


def connect_to_teensies(ports=None):
    """Return dict with {leg_number: teensy}"""
    if ports is None:
        tinfo = find_teensies()
        ports = [i['port'] for i in tinfo]
    teensies = [Teensy(p) for p in ports]
    lnd = {}
    for t in teensies:
        ln = t.leg_number
        lnd[ln] = lnd.get(ln, []) + [t, ]
    for ln in lnd:
        if len(lnd[ln]) > 1:
            raise Exception(
                "Found >1 teensies with the same leg number: %s[%s]" %
                (ln, lnd[ln]))
        lnd[ln] = lnd[ln][0]
    return lnd
