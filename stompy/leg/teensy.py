#!/usr/bin/env python

import glob
import logging
import subprocess
import threading
import time

import serial

import pycomando

from . import consts


logger = logging.getLogger(__name__)

cmds = {
    0: 'estop(byte)',  # 0 = off, 1 = soft, 2 = hard
    1: 'heartbeat',
    2: 'pwm(float,float,float)',  # hip thigh knee
    3: 'adc=uint32,uint32,uint32,uint32',
    4: 'adc_target(uint32,uint32,uint32)',
    5: 'pwm_value=int32,int32,int32',
    6: 'pid=float,float,float,float,float,float,float,float,float',
    7: 'plan(byte,byte,float,float,float,float,float,float,float)',
    8: 'enable_pid(bool)',
    9: 'xyz_values=float,float,float',
    10: 'angles=float,float,float,float,bool',
    11: 'set_pid(byte,float,float,float,float,float)',
    12: 'loop_time=uint32',
    13: 'leg_number(byte)=byte',
    14: 'pwm_limits(byte,float,float,float,float)',
    15: 'adc_limits(byte,float,float)',
    16: 'calf_scale(float,float)',
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


class Teensy(object):
    def __init__(self, port):
        self.port = port
        self.com = pycomando.Comando(serial.Serial(self.port, 9600))
        self.cmd = pycomando.protocols.command.CommandProtocol()
        self.com.register_protocol(0, self.cmd)
        # used for callbacks
        self.mgr = pycomando.protocols.command.EventManager(self.cmd, cmds)
        # easier for calling
        self.ns = self.mgr.build_namespace()
        # get leg number
        logger.debug("%s Get leg number" % port)
        self.leg_number = self.mgr.blocking_trigger('leg_number')[0].value
        logger.debug("%s leg number = %s" % (port, self.leg_number))

        self.leg_name = consts.LEG_NAMES_BY_NUMBER[self.leg_number]

        # disable leg
        self.ns.estop(consts.ESTOP_DEFAULT)
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

        self.mgr.on('xyz_values', self.on_xyz_values)
        self.mgr.on('angles', self.on_angles)

    def on_xyz_values(self, x, y, z):
        self.xyz = {
            'x': x.value, 'y': y.value, 'z': z.value,
            'time': time.time()}

    def on_angles(self, h, t, k, c, v):
        self.angles = {
            'hip': h.value, 'thigh': t.value, 'knee': k.value,
            'calf': c.value,
            'valid': bool(v), 'time': time.time()}

    def send_heartbeat(self):
        self.ns.heartbeat()
        self.last_heartbeat = time.time()

    def update(self):
        self.com.handle_stream()
        if time.time() - self.last_heartbeat > consts.HEARTBEAT_PERIOD:
            self.send_heartbeat()

    def _update_thread_function(self):
        while True:
            self.update()
            time.sleep(0.01)

    def start_update_thread(self):
        self._update_thread = threading.Thread(
            target=self._update_thread_function)
        self._update_thread.daemon = True
        self._update_thread.start()


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
