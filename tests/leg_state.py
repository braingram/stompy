#!/usr/bin/env python

import stompy.kinematics.leg as leg

import rospy
import sensor_msgs.msg

global joints
joints = None


def new_joints(data):
    global joints
    joints = data
    jd = dict(zip(joints.name, joints.position))
    # parse joint
    h = jd['stompyleg__body_to_fl']
    t = jd['stompyleg__fl__hip_to_thigh']
    c = jd['stompyleg__fl__thigh_to_calf_upper']
    s = jd['stompyleg__fl__calf_upper_to_calf_lower']
    x, y, z = leg.forward(h, t, c)
    print("Foot position: %s, %s, %s" % (x, y, z))
    l = s * 105076.1021919  # convert length to N
    print("Shock load: %s N" % l)


if __name__ == '__main__':
    rospy.init_node('leg_printer', anonymous=True)
    rospy.Subscriber(
        "/stompyleg/joint_states", sensor_msgs.msg.JointState, new_joints)
    rospy.spin()
