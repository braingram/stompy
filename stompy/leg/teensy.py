#!/usr/bin/env python

#import glob
import logging
#import subprocess
import sys
import time
import traceback

import numpy
import serial

import pycomando

from .. import consts
from .. import calibration
from .. import geometry
from .. import kinematics
from .. import log
from . import plans
from .. import signaler
from .. import transforms
from .. import utils


logger = logging.getLogger(__name__)

cmds = {
    0: 'heartbeat',
    1: 'estop(byte)=byte',  # 0 = off, 1 = soft, 2 = hard
    2: 'pwm(float,float,float)=float,float,float',  # hip thigh knee
    3: 'plan(byte,byte,float,float,float,float,float,float,float,'
       'float,float,float,float,float,float,float,float,float,float)',
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
    14: 'following_error_threshold(byte,float)=float',

    21: 'report_adc(bool)=uint32,uint32,uint32,uint32',
    22: 'report_pid(bool)='
        'float,float,float,float,float,float,float,float,float',
    23: 'report_pwm(bool)=int32,int32,int32',
    24: 'report_xyz(bool)=float,float,float',
    25: 'report_angles(bool)=float,float,float,float,bool',
    26: 'report_loop_time(bool)=uint32',
}

"""
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
        # TODO ignore body computer
        tinfo.append({
            'port': i['port'],
            'serial': i['ID_SERIAL_SHORT']})
    return tinfo
"""


class LegController(signaler.Signaler):
    def __init__(self, leg_number):
        super(LegController, self).__init__()
        self.leg_number = leg_number
        #log.info({'leg_number': self.leg_number})
        logger.debug("leg number = %s" % (self.leg_number, ))

        self.leg_name = consts.LEG_NAME_BY_NUMBER[self.leg_number]
        #log.info({'leg_name': self.leg_name})

        self.log = log.make_logger(self.leg_name)

        self.estop = None

        self.adc = {}
        self.angles = {}
        self.xyz = {}
        self.pid = {}
        self.pwm = {}

    def set_estop(self, value):
        if value != self.estop:
            self.estop = value
            self.log.info({'estop': value})
            self.trigger('estop', value)

    def enable_pid(self, value):
        self.log.debug({'enable_pid': value})

    def update(self):
        pass

    def _pack_plan(self, *args, **kwargs):
        if len(args) == 0 and len(kwargs) == 0:
            plan = plans.stop()
        if len(args) == 1 and isinstance(args[0], plans.Plan):
            plan = args[0]
        else:
            plan = plans.Plan(*args, **kwargs)
        return plan.packed(self.leg_number)

    def set_pwm(self, hip, thigh, knee):
        self.log.info({'set_pwm': {
            'hip': hip, 'thigh': thigh, 'knee': knee}})
        self.trigger('set_pwm', (hip, thigh, knee))

    def send_plan(self, *args, **kwargs):
        pp = self._pack_plan(*args, **kwargs)
        self.log.info({'plan': pp})
        self.trigger('plan', pp)

    def stop(self):
        """Send stop plan"""
        self.send_plan(plans.stop())


class FakeTeensy(LegController):
    def __init__(self, leg_number):
        super(FakeTeensy, self).__init__(leg_number)
        self.on('plan', self._new_plan)

        self.estop = True
        self.pwm = {'hip': 0, 'thigh': 0, 'knee': 0, 'time': time.time()}
        self.pid = {
            'time': time.time(),
            'output': {'hip': 0, 'thigh': 0, 'knee': 0},
            'set_point': {'hip': 0, 'thigh': 0, 'knee': 0},
            'error': {'hip': 0, 'thigh': 0, 'knee': 0},
        }
        self.adc = {
            'time': time.time(), 'hip': 0, 'thigh': 0, 'knee': 0, 'calf': 0}
        self.angles = {
            'time': time.time(),
            'hip': 0, 'thigh': 0.312, 'knee': -0.904, 'calf': 0}
        x, y, z = list(kinematics.leg.angles_to_points(
            self.angles['hip'], self.angles['thigh'], self.angles['knee']))[-1]
        self.xyz = {
            'time': time.time(), 'x': x, 'y': y, 'z': z}
        self._last_update = time.time()
        self._plan = None

    def _new_plan(self, pp):
        # if body frame, plan packing converted to leg
        if pp[0] == consts.PLAN_STOP_MODE:
            m, f, s = pp
            p = plans.Plan(m, f, speed=s)
        elif pp[0] in (consts.PLAN_TARGET_MODE, consts.PLAN_VELOCITY_MODE):
            m, f, lx, ly, lz, s = pp
            p = plans.Plan(
                m, f, linear=(lx, ly, lz), speed=s)
        elif pp[0] == consts.PLAN_ARC_MODE:
            m, f, lx, ly, lz, ax, ay, az, s = pp
            p = plans.Plan(
                m, f, linear=(lx, ly, lz), angular=(ax, ay, az), speed=s)
        elif pp[0] == consts.PLAN_MATRIX_MODE:
            m = pp[0]
            f = pp[1]
            s = pp[-1]
            matrix = numpy.matrix(numpy.identity(4))
            matrix[:3, :] = numpy.matrix(numpy.reshape(pp[2:-1], (3, 4)))
            p = plans.Plan(m, f, matrix=matrix, speed=s)
        if m != consts.PLAN_STOP_MODE:
            if f != consts.PLAN_LEG_FRAME:
                raise NotImplementedError('fake following of non-leg plans')
        self._plan = p

    def _follow_plan(self, t, dt):
        self.xyz['time'] = t
        self.angles['time'] = t
        if self.estop or self._plan is None:
            return
        if self._plan.mode == consts.PLAN_STOP_MODE:
            return
        if self._plan.frame != consts.PLAN_LEG_FRAME:
            raise NotImplementedError('fake following of non-leg plans')
        if self._plan.mode == consts.PLAN_VELOCITY_MODE:
            lx, ly, lz = self._plan.linear
            self.xyz['x'] += lx * dt * self._plan.speed
            self.xyz['y'] += ly * dt * self._plan.speed
            self.xyz['z'] += lz * dt * self._plan.speed
        elif self._plan.mode == consts.PLAN_TARGET_MODE:
            tx, ty, tz = self._plan.linear
            lx = tx - self.xyz['x']
            ly = ty - self.xyz['y']
            lz = tz - self.xyz['z']
            l = ((lx * lx) + (ly * ly) + (lz * lz)) ** 0.5
            if l < (self._plan.speed * dt) or l < 0.01:
                self.xyz['x'] = tx
                self.xyz['y'] = ty
                self.xyz['z'] = tz
            else:
                lx /= l
                ly /= l
                lz /= l
                self.xyz['x'] += lx * dt * self._plan.speed
                self.xyz['y'] += ly * dt * self._plan.speed
                self.xyz['z'] += lz * dt * self._plan.speed
        elif self._plan.mode == consts.PLAN_ARC_MODE:
            lx, ly, lz = self._plan.linear
            ax, ay, az = self._plan.angular
            ax *= self._plan.speed * dt
            ay *= self._plan.speed * dt
            az *= self._plan.speed * dt
            T = transforms.rotation_about_point_3d(
                lx, ly, lz, ax, ay, az, degrees=False)
            #nx, ny, nz = transforms.transform_3d(
            #    T, self.xyz['x'], self.xyz['y'], self.xyz['z'])

            ddt = dt
            nx, ny, nz = self.xyz['x'], self.xyz['y'], self.xyz['z']
            while ddt > 0.004:
                #nx, ny, nz = transforms.transform_3d(
                #    self._plan.matrix, nx, ny, nz)
                nx, ny, nz = transforms.transform_3d(
                    T, self.xyz['x'], self.xyz['y'], self.xyz['z'])
                ddt -= 0.004

            self.xyz['x'] = nx
            self.xyz['y'] = ny
            self.xyz['z'] = nz
        elif self._plan.mode == consts.PLAN_MATRIX_MODE:
            # call many times if dt > 4 ms)
            #print("_follow_plan:", self._plan.matrix)
            ddt = dt
            nx, ny, nz = self.xyz['x'], self.xyz['y'], self.xyz['z']
            while ddt > 0.004:
                nx, ny, nz = transforms.transform_3d(
                    self._plan.matrix, nx, ny, nz)
                ddt -= 0.004
            #print("X:", self.xyz['x'], nx)
            self.xyz['x'] = nx
            self.xyz['y'] = ny
            self.xyz['z'] = nz
        hip, thigh, knee = kinematics.leg.point_to_angles(
            self.xyz['x'], self.xyz['y'], self.xyz['z'])
        # check if angles are in limits, if not, stop
        if self.leg_number in (2, 5):
            hmin = geometry.HIP_MIDDLE_MIN_ANGLE
            hmax = geometry.HIP_MIDDLE_MAX_ANGLE
        else:
            hmin = geometry.HIP_MIN_ANGLE
            hmax = geometry.HIP_MAX_ANGLE
        in_limits = True
        if hip < hmin:
            hip = hmin
            in_limits = False
        if hip > hmax:
            hip = hmax
            in_limits = False
        if thigh < geometry.THIGH_MIN_ANGLE:
            thigh = geometry.THIGH_MIN_ANGLE
            in_limits = False
        if thigh > geometry.THIGH_MAX_ANGLE:
            thigh = geometry.THIGH_MAX_ANGLE
            in_limits = False
        if knee < geometry.KNEE_MIN_ANGLE:
            knee = geometry.KNEE_MIN_ANGLE
            in_limits = False
        if knee > geometry.KNEE_MAX_ANGLE:
            knee = geometry.KNEE_MAX_ANGLE
            in_limits = False
        if not in_limits:
            x, y, z = list(
                kinematics.leg.angles_to_points(hip, thigh, knee))[-1]
            self.xyz['x'] = x
            self.xyz['y'] = y
            self.xyz['z'] = z
            # raise estop
            self.set_estop(consts.ESTOP_HOLD)
        # fake calf loading
        zl = max(-55, min(-50, self.xyz['z']))
        calf = -(zl + 50) * 400
        self.angles.update({
            'hip': hip, 'thigh': thigh, 'knee': knee, 'calf': calf})
        #print(self.leg_number, self._plan.mode, self.angles, self.xyz)
        # get angles from x, y, z
        #h, t, k = 0., 0., 0.
        #x, y, z = list(kinematics.leg.angles_to_points(h, t, k))
        #self.xyz.update({'x': x, 'y': y, 'z': z})
        #self.angles.update({'hip': h, 'thigh': t, 'knee': k})

    def update(self):
        t = time.time()
        dt = t - self._last_update
        if dt > 0.1:
            # follow plan, update angles
            self._follow_plan(t, dt)
            self.angles['time'] = t
            self.adc['time'] = t
            self.xyz.update({'time': t})

            # generate events:
            self.pwm['time'] = t
            self.pid['time'] = t
            self.trigger('adc', self.adc)
            self.trigger('pwm', self.pwm)
            self.trigger('pid', self.pid)
            self.trigger('angles', self.angles)
            self.trigger('xyz', self.xyz)
            self._last_update = t


class Teensy(LegController):
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
        ln = self.mgr.blocking_trigger('leg_number')[0].value
        super(Teensy, self).__init__(ln)

        self._text = pycomando.protocols.text.TextProtocol()

        def print_text(txt, leg_number=ln):
            print("DEBUG[%s]:%s" % (leg_number, txt))

        self._text.register_callback(print_text)
        self.com.register_protocol(1, self._text)

        # load calibration setup
        for v in calibration.setup.get(self.leg_number, []):
            self.log.debug({'calibration': v})
            f, args = v
            logger.debug("Calibration: %s, %s" % (f, args))
            self.mgr.trigger(f, *args)

        self.mgr.on('estop', self.on_estop)

        # disable leg
        self.set_estop(consts.ESTOP_DEFAULT)

        # send first heartbeat
        self.send_heartbeat()

        self.mgr.on('report_xyz', self.on_report_xyz)
        self.mgr.on('report_angles', self.on_report_angles)
        self.mgr.on('report_pid', self.on_report_pid)
        self.mgr.on('report_pwm', self.on_report_pwm)
        self.mgr.on('report_adc', self.on_report_adc)

    def on_estop(self, severity):
        #print("Received estop: %s" % severity)
        super(Teensy, self).set_estop(severity.value)

    def send_plan(self, *args, **kwargs):
        pp = self._pack_plan(*args, **kwargs)
        #print("plan: %s" % pp)
        self.log.info({'plan': pp})
        self.trigger('plan', pp)
        self.mgr.trigger('plan', *pp)

    def set_estop(self, value):
        self.mgr.trigger('estop', value)
        super(Teensy, self).set_estop(value)

    def set_pwm(self, hip, thigh, knee):
        self.mgr.trigger('pwm', hip, thigh, knee)
        super(Teensy, self).set_pwm(hip, thigh, knee)

    def enable_pid(self, value):
        self.mgr.trigger('enable_pid', value)
        super(Teensy, self).enable_pid(value)

    def on_report_adc(self, hip, thigh, knee, calf):
        self.adc = {
            'hip': hip.value, 'thigh': thigh.value,
            'knee': knee.value, 'calf': calf.value,
            'time': time.time()}
        self.log.debug({'adc': self.adc})
        self.trigger('adc', self.adc)

    def on_report_xyz(self, x, y, z):
        t = time.time()
        x, y, z = x.value, y.value, z.value
        self.xyz = {
            'x': x, 'y': y, 'z': z,
            'time': t}
        self.log.debug({'xyz': self.xyz})
        self.trigger('xyz', self.xyz)

    def on_report_angles(self, h, t, k, c, v):
        self.angles = {
            'hip': h.value, 'thigh': t.value, 'knee': k.value,
            'calf': c.value,
            'valid': bool(v), 'time': time.time()}
        self.log.debug({'angles': self.angles})
        self.trigger('angles', self.angles)

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
        self.log.debug({'pid': self.pid})
        self.trigger('pid', self.pid)

    def on_report_pwm(self, h, t, k):
        self.pwm = {
            'hip': h.value, 'thigh': t.value, 'knee': k.value,
            'time': time.time()}
        self.log.debug({'pwm': self.pwm})
        self.trigger('pwm', self.pwm)

    def send_heartbeat(self):
        self.mgr.trigger('heartbeat')
        self.last_heartbeat = time.time()
        # print("HB: %s" % self.last_heartbeat)

    def update(self):
        try:
            self.com.handle_stream()
        except Exception as e:
            ex_type, ex, tb = sys.exc_info()
            print("Leg %s handle stream error: %s" % (self.leg_number, e))
            tbs = '\n'.join(traceback.format_tb(tb))
            self.log.error("handle_stream error: %s" % e)
            print(tbs)
            self.log.error({'error': {
                'traceback': tbs,
                'exception': e}})
            raise e
        if time.time() - self.last_heartbeat > consts.HEARTBEAT_PERIOD:
            self.send_heartbeat()


def connect_to_teensies(ports=None):
    """Return dict with {leg_number: teensy}"""
    if ports is None:
        tinfo = utils.find_leg_teensies()
        ports = [i['port'] for i in tinfo]
    if len(ports) == 0:
        return {ln: FakeTeensy(ln) for ln in [1, 2, 3, 4, 5, 6]}
        #return {ln: FakeTeensy(ln) for ln in [1, 3, 4, 6]}
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
