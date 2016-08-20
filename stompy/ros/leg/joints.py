#!/usr/bin/env python

import rospy
import sensor_msgs.msg


global_defaults = {
    'hip': 0.,
    'thigh': 0.,
    'knee': 0.,
    'calf': 0.,
    'ankle': 0.,
    'foot': 0.,
    'pad': 0.,
}


class JointStatePublisher(object):
    def __init__(self, name, defaults=None, connect=True):
        self.name = name
        if defaults is None:
            self.defaults = global_defaults
        else:
            self.defaults = defaults
        self.pub = None
        if connect:
            self.connect()

    def connect(self, queue_size=10):
        self.pub = rospy.Publisher(
            '/stompy/joint_states', sensor_msgs.msg.JointState,
            queue_size=queue_size)

    def publish(self, **joints):
        msg = sensor_msgs.msg.JointState()
        js = self.defaults.copy()
        js.update(joints)
        for j in js:
            msg.name.append('%s_%s' % (self.name, j))
            msg.position.append(js[j])
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
