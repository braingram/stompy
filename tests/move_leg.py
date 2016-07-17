#!/usr/bin/env python

import argparse
import numpy

import rospy
import std_msgs.msg


import stompy.kinematics.leg as leg


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-x', '--x', default=0., type=float)
    p.add_argument('-y', '--y', default=0., type=float)
    p.add_argument('-z', '--z', default=0., type=float)
    p.add_argument('-X', '--dx', default=0., type=float)
    p.add_argument('-Y', '--dy', default=0., type=float)
    p.add_argument('-Z', '--dz', default=0., type=float)
    p.add_argument('-r', '--raw', default=False, action='store_true')
    args = p.parse_args()

    # xy looking down
    x = args.x
    y = args.y
    z = args.z

    compute = not args.raw

    lc = lambda joint: '/stompyleg/%s_controller/command' % (joint, )
    fm = lambda data: std_msgs.msg.Float64(data)

    if compute:
        hip_angle, thigh_angle, knee_angle = leg.inverse(x, y, z)
    else:
        hip_angle = x
        thigh_angle = y
        knee_angle = z

    rospy.init_node('stompyleg_move', anonymous=True)
    hp = rospy.Publisher(lc('hip'), std_msgs.msg.Float64, queue_size=18)
    tp = rospy.Publisher(lc('thigh'), std_msgs.msg.Float64, queue_size=18)
    kp = rospy.Publisher(lc('knee'), std_msgs.msg.Float64, queue_size=18)

    dx = args.dx
    dy = args.dy
    dz = args.dz
    while not rospy.is_shutdown():
        print(
            "Hip: %s, Thigh: %s, Knee: %s" % (
                hip_angle, thigh_angle, knee_angle))
        hp.publish(hip_angle)
        tp.publish(thigh_angle)
        kp.publish(knee_angle)
        x += dx
        y += dy
        z += dz
        if compute:
            hip_angle, thigh_angle, knee_angle = leg.inverse(x, y, z)
        else:
            hip_angle = x
            thigh_angle = y
            knee_angle = z
        at_limit = False
        if (hip_angle < leg.hip_limits[0] or hip_angle > leg.hip_limits[1]):
            print("Hit hip limit")
            at_limit = True
        if (
                thigh_angle < leg.thigh_limits[0] or
                thigh_angle > leg.thigh_limits[1]):
            print("Hit thigh limit")
            at_limit = True
        if (
                knee_angle < leg.knee_limits[0] or
                knee_angle > leg.knee_limits[1]):
            print("Hit knee limit")
            at_limit = True
        if at_limit:
            dx *= -1
            dy *= -1
            dz *= -1
            x += dx
            y += dy
            z += dz

        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
