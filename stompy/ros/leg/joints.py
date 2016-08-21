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
    print("joints connecting to ros")
    pub = rospy.Publisher(
        '/stompy/joint_states', sensor_msgs.msg.JointState,
        queue_size=10)


def connect_to_teensy():
    print("joints connecting to teensy")
    teensy.mgr.on('joints', new_joints)


def connect():
    connect_to_ros()
    connect_to_teensy()


def new_joints(time, hip, thigh, knee, calf):
    # TODO convert teensy time to ros?
    send_joints(
        None, hip=hip.value, thigh=thigh.value,
        knee=knee.value, calf=calf.value)


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
