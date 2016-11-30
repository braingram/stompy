#!/usr/bin/env python

from .. import callbacker

global states
states = {}

callbacks = {
    'fr': callbacker.Callbacker(),
    'fl': callbacker.Callbacker(),
    'mr': callbacker.Callbacker(),
    'ml': callbacker.Callbacker(),
    'rr': callbacker.Callbacker(),
    'rl': callbacker.Callbacker(),
}


def decode_state_point(pt, jns):
    return {
        'positions': dict(zip(jns, pt.positions)),
        'velocities': dict(zip(jns, pt.velocities)),
        #'accelerations': dict(zip(jns, pt.accelerations)),
        #'effort': dict(zip(jns, pt.effort)),
        #'time_from_start': pt.time_from_start,
    }


def update_state(leg, msg):
    global states
    t = msg.header.stamp
    jns = [jn.split('_')[1] for jn in msg.joint_names]
    states[leg] = {
        'time': t,
        'desired': decode_state_point(msg.desired, jns),
        'actual': decode_state_point(msg.actual, jns),
        'error': decode_state_point(msg.error, jns),
    }
    callbacks[leg](states)
