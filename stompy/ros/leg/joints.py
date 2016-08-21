#!/usr/bin/env python

import rospy
import sensor_msgs.msg

from . import info
from ...leg import teensy


defaults = {
    'hip': 0.,
    'thigh': 0.,
    'knee': 0.,
    'calf': 0.,
    'ankle': 0.,
    'foot': 0.,
    'pad': 0.,
}

global pub
pub = None


def connect_to_ros():
    global pub
    pub = rospy.Publisher(
        '/stompy/joint_states', sensor_msgs.msg.JointState,
        queue_size=10)


def connect_to_teensy():
    teensy.mgr.on('joints', new_joints)


def connect():
    connect_to_teensy()
    connect_to_ros()


def new_joints(self, time, hip, thigh, knee, calf):
    # TODO convert teensy time to ros?
    self.send_joints(None, hip=hip, thigh=thigh, knee=knee, calf=calf)


def send_joints(time=None, **joints):
    msg = sensor_msgs.msg.JointState()
    js = defaults.copy()
    js.update(joints)
    for j in js:
        msg.name.append('%s_%s' % (info.name, j))
        msg.position.append(js[j])
    if time is None:
        msg.header.stamp = rospy.Time.now()
    else:
        # TODO convert time from teensy?
        raise NotImplementedError
    pub.publish(msg)
