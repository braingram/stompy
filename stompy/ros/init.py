#!/usr/bin/env python

import rospy

from . import clock
from . import controllers
from . import heart
from . import joints
from . import joystick
from . import legs


def init(name=None):
    if name is None:
        name = 'stompy_control'
    rospy.init_node(name, anonymous=True)
    heart.connect()
    joints.connect_to_joint_states()
    controllers.connect_to_leg_controllers()
    joystick.connect_to_joystick()
    legs.connect_to_leg_publishers()
    clock.connect()
