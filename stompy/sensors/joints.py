#!/usr/bin/env python

from .. import callbacker


global joints
joints = None

callbacks = callbacker.Callbacker()


def update_joints(new_joints, timestamp):
    global joints
    if joints is None:
        joints = {}
    for name in new_joints:
        joints[name] = (new_joints[name], timestamp)
    callbacks(joints)
