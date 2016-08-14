#!/usr/bin/env python

from .. import callbacker

global time
time = None

callbacks = callbacker.Callbacker()


def update_time(new_time):
    global time
    time = new_time
    callbacks(time)
