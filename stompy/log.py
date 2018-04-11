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


def load_dir(d):
    fns = glob.glob(os.path.join(d, '*.p'))
    fns = sorted(
        fns, key=lambda fn: int(
            os.path.splitext(os.path.basename(fn))[0].split('_')[0]))
    evs = []
    for fn in fns:
        with open(fn, 'rb') as f:
            evs.extend(pickle.load(f))
    return evs


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
    'stompy_logs', start_time.strftime('%y%m%d_%H%M%S'))
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
