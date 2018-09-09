#!/usr/bin/env python

import glob
import subprocess


# serial numbers of body teensies
body_teensies = [
    '2513500',
    '2595730'
]


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
