#!/usr/bin/env python

import rospy

from . import joints


def init_leg(name=None):
    if name is None:
        name = 'fr'
    rospy.init_node(name, anonymous=True)
    # connect to master
    # setup joint publishers
    # setup trajectory action server


def fake_joints(name=None):
    if name is None:
        name = 'fr'
    rospy.init_node(name + '_fake_joints', anonymous=True)
    jp = joints.JointStatePublisher(name)
    while not rospy.is_shutdown():
        jp.publish()
        rospy.sleep(0.1)
