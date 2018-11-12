#!/usr/bin/env python

#import glob
import os
#import subprocess

import teensyloader


firmware_dir = os.path.abspath(os.path.expanduser('~/.stompy/hex/'))

teensy_serial_by_type = {
    'body': [
        '2513500',
        '2595730'
    ],
    'leg': [
        '1364130',  # fake leg, don't auto-program this
	'1890580',  # leg 1
	'1960390',  # leg 2
	'2084140',  # leg 3
	'1890640',  # leg 4
	'2052790',  # leg 5
	'2619770',  # leg 6
    ],
}

teensy_type_by_serial = {
    s: t for t in teensy_serial_by_type for s in teensy_serial_by_type[t]}


def find_teensies_by_type(teensy_type=None):
    by_serial = teensyloader.find_serial_teensies()
    teensies_by_type = {}
    for s in by_serial:
        # make path
        port_fn = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_%s-if00' % s
        if os.path.exists(port_fn):
            port_fn = os.path.realpath(port_fn)
        else:
            # throw error here?
            port_fn = None
        t = teensy_type_by_serial.get(s, 'unknown')
        teensies_by_type[t] = teensies_by_type.get(t, []) + [
            {'serial': s, 'port': port_fn, 'info': by_serial[s]}, ]
    if teensy_type is None:
        return teensies_by_type
    return teensies_by_type.get(teensy_type, [])


def get_firmware_hex_path(teensy_type):
    """Return path to hex file for this teensy type or None"""
    hex_filename = os.path.join(firmware_dir, teensy_type + '.hex')
    if os.path.isfile(hex_filename):
        return hex_filename
    return None


def program_teensies_by_type(teensy_types=None):
    ts = find_teensies_by_type()
    print("Found teensies: %s" % (ts, ))
    if teensy_types is None:
        teensy_types = ts.keys()
    for t in teensy_types:
        hex_fn = get_firmware_hex_path(t)
        if hex_fn is None:
            print("No hex file found for %s" % (t, ))
            continue
        # program this teensy with this type
        for teensy_info in ts[t]:
            serial = teensy_info['serial']
            print("Programming: %s with %s" % (serial, hex_fn))
            teensyloader.program_teensy(
                hex_fn, mcu="TEENSY32",
                dev=serial, autoboot=True)
    return


"""
# serial numbers of body teensies
body_teensies = [
    '2513500',
    '2595730'
]


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


def find_leg_teensies():
    return [
        t for t in find_teensies() if t['serial'] not in body_teensies]


def find_body_teensies():
    return [
        t for t in find_teensies() if t['serial'] in body_teensies]
"""


class StatsMonitor(object):
    def __init__(self):
        self.reset()

    def update(self, v):
        if self.n == 0:
            self.min = v
            self.max = v
        else:
            self.min = min(v, self.min)
            self.max = max(v, self.max)
        self.sum += v
        self.n += 1

    def reset(self):
        self.min = float('nan')
        self.max = float('nan')
        self.std = float('nan')
        self.sum = 0
        self.n = 0

    @property
    def mean(self):
        if self.n == 0:
            return float('nan')
        return self.sum / float(self.n)

    def __str__(self):
        return "%s[n=%i, mean=%.2g, min=%.2g, max=%.2g]" % (
            self.__class__.__name__, self.n, self.mean, self.min, self.max)
