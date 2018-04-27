#!/usr/bin/env python
"""
Write timestamped dicts to pickle file
"""

import atexit
import datetime
import glob
import logging
import os
import cPickle as pickle
import time

import pylab


def find_newest_log():
    ld = os.path.expanduser('~/.stompy/logs')
    return sorted([
        os.path.join(ld, d) for d in os.listdir(ld)
        if os.path.isdir(os.path.join(ld, d))])[-1]


def load_dir(d=None):
    if d is None:
        d = find_newest_log()
    ld = os.path.expanduser(d)
    sds = [
        i for i in os.listdir(ld)
        if os.path.isdir(os.path.join(ld, i))]
    # update for per-leg logs
    if len(sds):
        return {sd: load_dir(os.path.join(ld, sd)) for sd in sds}
    fns = glob.glob(os.path.join(d, '*.p'))
    fns = sorted(
        fns, key=lambda fn: int(
            os.path.splitext(os.path.basename(fn))[0].split('_')[0]))
    evs = []
    for fn in fns:
        with open(fn, 'rb') as f:
            evs.extend(pickle.load(f))
    return evs


def _dget(d, k):
    if '.' not in k:
        return d[k]
    ts = k.split('.')
    return _dget(d[ts[0]], '.'.join(ts[1:]))


get_by_key = lambda d, k: [i[k] for i in d if k in i]


def plot_key(data, key, subkeys=None, show=True, name=None, legend=True):
    if isinstance(data, (str, unicode)):
        if name is None:
            name = data
        data = load_dir(data)
    legs = sorted(data.keys())
    if 'base' in legs:
        legs.remove('base')
    ld = {l: get_by_key(data[l], key) for l in legs}
    if subkeys is None:
        # get keys from first event
        e = ld.values()[0][0]
        ks = e.keys()
        if 'time' in ks:
            ks.remove('time')
        subkeys = []
        for k in ks:
            if isinstance(e[k], dict):
                for sk in e[k]:
                    subkeys.append('%s.%s' % (k, sk))
            else:
                subkeys.append(k)
        subkeys.sort()
        #print("Found subkeys: %s" % (subkeys, ))
    subkeys = list(subkeys)
    nsk = len(subkeys)
    for i in xrange(len(subkeys)):
        if i == 0:
            ax = pylab.subplot(nsk, 1, 1 + i)
        else:
            pylab.subplot(nsk, 1, 1 + i, sharex=ax)
        for l in legs:
            pylab.plot(
                [e['time'] for e in ld[l]],
                [_dget(e, subkeys[i]) for e in ld[l]], label=l)
        pylab.ylabel(subkeys[i])
    if name is not None:
        pylab.suptitle("%s: %s" % (name, key))
    else:
        pylab.suptitle(key)
    if legend:
        pylab.legend()
    if show:
        pylab.show()


def find_adc_limits(d=None, joints=None, legs=None):
    if isinstance(d, (str, unicode)) or d is None:
        d = load_dir(d)
    if joints is None:
        joints = ['hip', 'thigh', 'knee', 'calf']
    if legs is None:
        legs = d.keys()
        if 'base' in legs:
            legs.remove('base')
    adc_limits = {}
    for leg in legs:
        ld = d[leg]
        angles = [i['adc'] for i in ld if 'adc' in i]
        adc_limits[leg] = {}
        for j in joints:
            vs = [i[j] for i in angles]
            adc_limits[leg][j] = {
                'min': min(vs),
                'max': max(vs),
            }
    return adc_limits


class Logger(object):
    def __init__(self, directory, events_per_file=10000):
        self.level = logging.DEBUG
        self._dir = directory
        self._events = []
        self._file_index = 0
        self.events_per_file = events_per_file

    def _write_events(self):
        if len(self._events) == 0:
            return
        # write to disk
        if not os.path.exists(self._dir):
            os.makedirs(self._dir)
        # directory/index_timestamp.p
        fn = '%04i_%s.p' % (self._file_index, int(time.time()))
        fp = os.path.join(self._dir, fn)
        with open(fp, 'wb') as f:
            pickle.dump(self._events, f)
        self._events = []
        self._file_index += 1

    @property
    def directory(self):
        return self._dir

    @directory.setter
    def directory(self, directory):
        self._write_events()
        self._dir = directory

    def log(self, event, level):
        if level < self.level:
            return
        if not isinstance(event, dict):
            event = {'event': event}
        if 'timestamp' not in event:
            event['timestamp'] = time.time()
        self._events.append(event)
        if len(self._events) >= self.events_per_file:
            self._write_events()

    def critical(self, event):
        self.log(event, logging.CRITICAL)

    def error(self, event):
        self.log(event, logging.ERROR)

    def warning(self, event):
        self.log(event, logging.WARNING)

    def info(self, event):
        self.log(event, logging.INFO)

    def debug(self, event):
        self.log(event, logging.DEBUG)

start_time = datetime.datetime.now()
log_directory = os.path.join(
    os.path.expanduser('~'),
    '.stompy', 'logs',
    start_time.strftime('%y%m%d_%H%M%S'))
base_log_directory = os.path.join(log_directory, 'base')

logger = Logger(base_log_directory)

critical = logger.critical
error = logger.error
warning = logger.warning
info = logger.info
debug = logger.debug

atexit.register(logger._write_events)


def make_logger(name):
    ldir = os.path.join(log_directory, name)
    print("Making logger: %s" % ldir)
    l = Logger(ldir)
    atexit.register(l._write_events)
    return l
