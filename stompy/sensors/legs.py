#!/usr/bin/env python

from .. import callbacker
from .. import kinematics

from . import joints


global legs
legs = {}

callbacks = callbacker.Callbacker()


def build_description(leg_name):
    return {
        'hip': 'stompy__body_to_%s' % (leg_name),
        'thigh': 'stompy__%s__hip_to_thigh' % (leg_name),
        'knee': 'stompy__%s__thigh_to_calf_upper' % (leg_name),
        'calf': 'stompy__%s__calf_upper_to_calf_lower' % (leg_name),
    }


descriptions = {
    'fl': build_description('fl'),
    'fr': build_description('fr'),
    'ml': build_description('ml'),
    'mr': build_description('mr'),
    'rl': build_description('rl'),
    'rr': build_description('rr'),
}


def update_legs(new_joints):
    global legs
    for leg_name in descriptions:
        for joint_name in descriptions[leg_name]:
            joint_key = descriptions[leg_name][joint_name]
            if joint_key in new_joints:
                if leg_name not in legs:
                    legs[leg_name] = {}
                legs[leg_name][joint_name] = new_joints[joint_key][0]
                legs[leg_name]['timestamp'] = new_joints[joint_key][1]
        # compute foot positions here
        if leg_name in legs:
            leg = legs[leg_name]
            if 'hip' in leg and 'thigh' in leg and 'knee' in leg:
                leg['foot'] = kinematics.leg.forward(
                    leg['hip'], leg['thigh'], leg['knee'])
            if 'calf' in leg:
                # compute leg loads here
                # convert from meters to newtons to lbs
                leg['load'] = leg['calf'] * 105075.9 * 0.224808942443
    callbacks(legs)


joints.callbacks.register(update_legs)
