#!/usr/bin/env python
"""
read in joints
find foot positions
move all feet down/up until body is at z
"""

import argparse

import rospy
import sensor_msgs.msg
import std_msgs.msg


import stompy.kinematics.leg as leg
from stompy.planners.trajectory import PathTracer
import stompy.sensors.joints as joints


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-z', '--z', default=1.0, type=float)
    p.add_argument('-x', '--x', default=0.0, type=float)
    p.add_argument('-t', '--time', default=5., type=float)
    p.add_argument('-r', '--rate', default=10., type=float)
    args = p.parse_args()

    # subscribe to joint messages
    rospy.init_node('stompy_stand', anonymous=True)
    rospy.Subscriber(
        "/stompy/joint_states", sensor_msgs.msg.JointState,
        lambda data, description=joints.stompy_leg_descriptions:
        joints.update_joints(data, description))

    # setup publishers for commanding joints
    lc = lambda leg, joint: '/stompy/%s_%s_controller/command' % (leg, joint, )
    leg_publishers = {}
    foot_paths = {}
    for leg_name in ('fl', 'fr', 'ml', 'mr', 'rl', 'rr'):
        leg_publishers[leg_name] = {}
        foot_paths[leg_name] = PathTracer(time=args.time, rate=args.rate)
        for joint_name in ('hip', 'thigh', 'knee'):
            leg_publishers[leg_name][joint_name] = rospy.Publisher(
                lc(leg_name, joint_name), std_msgs.msg.Float64, queue_size=18)

    state = 0  # awaiting joints
    sleep_time = 1. / args.rate
    while not rospy.is_shutdown():
        if state == 0:  # awaiting joints
            if joints.joints is not None:
                # setup paths using computed start points
                for leg_name in joints.legs:
                    # compute foot position
                    foot = leg.forward(
                        joints.legs[leg_name]['hip'],
                        joints.legs[leg_name]['thigh'],
                        joints.legs[leg_name]['calf'])
                    # setup path for foot
                    foot_paths[leg_name].start = foot
                    ep = [foot[0], foot[1], args.z]
                    foot_paths[leg_name].end = ep
                    print(
                        "Preparing %s foot move from %s to %s" %
                        (leg_name, foot, ep))
                state = 1  # moving feet
            else:
                print("Awaiting joints")
        elif state == 1:  # moving feet
            # execute paths
            state = 2
            for leg_name in foot_paths:
                path = foot_paths[leg_name]
                pub = leg_publishers[leg_name]
                pt = path.next()
                if pt is None:
                    continue
                state = 1
                h, t, k = leg.inverse(pt[0], pt[1], pt[2])
                pub['hip'].publish(h)
                pub['thigh'].publish(t)
                pub['knee'].publish(k)
        else:
            print("Done moving")
            break
        rospy.sleep(sleep_time)


if __name__ == '__main__':
    main()
