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
import stompy.kinematics.body as body
from stompy.planners.trajectory import PathTracer
import stompy.sensors.joints as joints


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-z', '--sz', default=0.5, type=float)
    p.add_argument('-Z', '--ez', default=1.2, type=float)
    p.add_argument('-x', '--x', default=1.5, type=float)
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
    target_axis = 2
    moves = 0
    target = args.sz
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
                        joints.legs[leg_name]['knee'])
                    # setup path for foot
                    foot_paths[leg_name].start = foot
                    ep = [foot[0], 0., foot[2]]
                    ep[target_axis] = target
                    h, t, k = leg.inverse(*ep)
                    if not any(
                            (
                                leg.in_limits(h, leg.hip_limits),
                                leg.in_limits(t, leg.thigh_limits),
                                leg.in_limits(k, leg.knee_limits),
                            )):
                        raise Exception()
                    foot_paths[leg_name].end = ep
                    print(
                        "Preparing %s foot move from %s to %s" %
                        (leg_name, foot, ep))
                state = 1  # moving feet
            else:
                print("Awaiting joints")
        elif state == 1:  # moving feet
            # execute paths
            state = -1
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
            if state == -1:
                for leg_name in ('fl', 'fr', 'ml', 'mr', 'rl', 'rr'):
                    foot_paths[leg_name] = PathTracer(
                        time=args.time, rate=args.rate)
                if moves == 0:
                    target_axis = 0
                    target = args.x
                    state = 0
                elif moves == 1:
                    target_axis = 2
                    target = args.ez
                    state = 0
                    # moves = 7  # skip to leg lifts
                elif moves == 2:
                    # start body moves
                    target_axis = 0
                    target = 0.5
                    state = 2
                elif moves == 3:
                    target_axis = 0
                    target = -1.0
                    state = 2
                elif moves == 4:
                    target_axis = 0
                    target = 0.5
                    state = 2
                elif moves == 5:
                    target_axis = 1
                    target = 0.5
                    state = 2
                elif moves == 6:
                    target_axis = 1
                    target = -1.0
                    state = 2
                elif moves == 7:
                    target_axis = 1
                    target = 0.5
                    state = 2
                elif moves == 8:  # leg lifts
                    target_axis = 'fl'
                    target = 0.2
                    state = 3
                elif moves == 9:
                    target_axis = 'fl'
                    target = args.ez
                    state = 3
                elif moves == 10:
                    target_axis = 'fr'
                    target = 0.2
                    state = 3
                elif moves == 11:
                    target_axis = 'fr'
                    target = args.ez
                    state = 3
                elif moves == 12:
                    target_axis = 'ml'
                    target = 0.2
                    state = 3
                elif moves == 13:
                    target_axis = 'ml'
                    target = args.ez
                    state = 3
                elif moves == 14:
                    target_axis = 'mr'
                    target = 0.2
                    state = 3
                elif moves == 15:
                    target_axis = 'mr'
                    target = args.ez
                    state = 3
                elif moves == 16:
                    target_axis = 'rl'
                    target = 0.2
                    state = 3
                elif moves == 17:
                    target_axis = 'rl'
                    target = args.ez
                    state = 3
                elif moves == 18:
                    target_axis = 'rr'
                    target = 0.2
                    state = 3
                elif moves == 19:
                    target_axis = 'rr'
                    target = args.ez
                    state = 3

                moves += 1
        elif state == 2:  # move body
            if joints.joints is not None:
                # setup paths using computed start points
                for leg_name in joints.legs:
                    # compute foot position
                    foot = leg.forward(
                        joints.legs[leg_name]['hip'],
                        joints.legs[leg_name]['thigh'],
                        joints.legs[leg_name]['knee'])
                    # compute in body coordinates
                    p = list(body.leg_to_body(leg_name, *foot))
                    p[target_axis] += target
                    f = body.body_to_leg(leg_name, *p)

                    # setup path for foot
                    foot_paths[leg_name].start = foot
                    foot_paths[leg_name].end = f
                    print(
                        "Preparing %s foot move from %s to %s" %
                        (leg_name, foot, ep))
                state = 1  # moving feet
            else:
                print("Awaiting joints")
        elif state == 3:  # lift leg
            if joints.joints is not None:
                # setup paths using computed start points
                for leg_name in joints.legs:
                    # compute foot position
                    foot = leg.forward(
                        joints.legs[leg_name]['hip'],
                        joints.legs[leg_name]['thigh'],
                        joints.legs[leg_name]['knee'])
                    foot_paths[leg_name].start = foot
                    if leg_name == target_axis:
                        ep = [foot[0], foot[1], target]
                    else:
                        ep = foot
                    h, t, k = leg.inverse(*ep)
                    if not any(
                            (
                                leg.in_limits(h, leg.hip_limits),
                                leg.in_limits(t, leg.thigh_limits),
                                leg.in_limits(k, leg.knee_limits),
                            )):
                        raise Exception()
                    foot_paths[leg_name].end = ep
                    print(
                        "Preparing %s foot move from %s to %s" %
                        (leg_name, foot, ep))
                state = 1  # moving feet
            else:
                print("Awaiting joints")
        else:
            print("Done moving")
            break
        rospy.sleep(sleep_time)


if __name__ == '__main__':
    main()
