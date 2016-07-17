#!/usr/bin/env python

from .. import kinematics


global joints
joints = None

global legs
legs = {}


def build_description(leg_name):
    return {
        'hip': 'stompy__body_to_%s' % (leg_name),
        'thigh': 'stompy__%s__hip_to_thigh' % (leg_name),
        'knee': 'stompy__%s__thigh_to_calf_upper' % (leg_name),
        'shock': 'stompy__%s__calf_upper_to_calf_lower' % (leg_name),
    }


stompy_leg_descriptions = {
    'fl': build_description('fl'),
    'fr': build_description('fr'),
    'ml': build_description('ml'),
    'mr': build_description('mr'),
    'rl': build_description('rl'),
    'rr': build_description('rr'),
}


def update_joints(data, leg_descriptions=None):
    """
    rospy.init_node('stompyleg_trace', anonymous=True)
    rospy.Subscriber(
        "/stompyleg/joint_states", sensor_msgs.msg.JointState, update_joints)
    """
    global joints
    if joints is None:
        joints = {}
    joints.update(dict(zip(data.name, data.position)))
    # TODO deal with different headers for different legs
    joints['header'] = data.header
    if leg_descriptions is None:
        return
    global legs
    for leg_name in leg_descriptions:
        for joint_name in leg_descriptions[leg_name]:
            joint_key = leg_descriptions[leg_name][joint_name]
            if joint_key in joints:
                if leg_name not in legs:
                    legs[leg_name] = {}
                legs[leg_name]['header'] = data.header
                legs[leg_name][joint_name] = joints[joint_key]
        # compute foot positions here
        if leg_name in legs:
            leg = legs[leg_name]
            if 'hip' in leg and 'thigh' in leg and 'knee' in leg:
                leg['foot'] = kinematics.leg.forward(
                    leg['hip'], leg['thigh'], leg['knee'])
