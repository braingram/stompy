#!/usr/bin/env python
"""
Trace a path with a leg
  start position (x, y, z) or blank to read out
  end position (x, y, z)
  time to move (seconds)
  rate to update (hz)


(read out start position)
compute vector from start to end
break up vector wrt time and rate
start movement...
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
    p.add_argument('-s', '--start', default="", type=str)
    p.add_argument('-e', '--end', default="1.8,0.0,0.5", type=str)
    p.add_argument('-t', '--time', default=5., type=float)
    p.add_argument('-r', '--rate', default=10., type=float)
    args = p.parse_args()

    rospy.init_node('stompyleg_trace', anonymous=True)
    rospy.Subscriber(
        "/stompyleg/joint_states", sensor_msgs.msg.JointState,
        lambda data, description=joints.stompyleg_leg_descriptions:
        joints.update_joints(data, description))

    lc = lambda joint: '/stompyleg/%s_controller/command' % (joint, )

    hp = rospy.Publisher(lc('hip'), std_msgs.msg.Float64, queue_size=18)
    tp = rospy.Publisher(lc('thigh'), std_msgs.msg.Float64, queue_size=18)
    kp = rospy.Publisher(lc('knee'), std_msgs.msg.Float64, queue_size=18)

    start = None
    if args.start != "":
        start = map(float, args.start.split(','))
    end = map(float, args.end.split(','))
    path = PathTracer(start=start, end=end, time=args.time, rate=args.rate)
    pt = None
    print("Target: %s" % end)
    sleep_time = 1. / args.rate
    while not rospy.is_shutdown():
        if path.start is None:
            # (read out start position)
            if joints.joints is not None:
                path.start = leg.forward(
                    joints.legs['fl']['hip'],
                    joints.legs['fl']['thigh'],
                    joints.legs['fl']['calf'])
                print("Found starting position: %s" % (path.start, ))
        else:
            # compute vector from start to end
            # break up vector wrt time and rate
            pt = path.next()
            if pt is None:
                break
            print("Moving to: %s" % (pt, ))
            h, t, k = leg.inverse(pt[0], pt[1], pt[2])
            hp.publish(h)
            tp.publish(t)
            kp.publish(k)
            # start movement...
        rospy.sleep(sleep_time)

if __name__ == '__main__':
    main()
