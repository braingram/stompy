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
import numpy

import rospy
import sensor_msgs.msg
import std_msgs.msg


import stompy.kinematics.leg as leg


global joints
joints = None


def new_joints(data):
    global joints
    joints = dict(zip(data.name, data.position))
    joints['header'] = data.header
    joints['hip'] = joints['stompyleg__body_to_fl']
    joints['thigh'] = joints['stompyleg__fl__hip_to_thigh']
    joints['calf'] = joints['stompyleg__fl__thigh_to_calf_upper']
    joints['shock'] = joints['stompyleg__fl__calf_upper_to_calf_lower']


class PathTracer(object):
    def __init__(self, start=None, end=None, time=None, rate=None):
        self.start = start
        self.end = end
        self.time = time
        self.rate = rate
        self.i = None
        self.n = None

    def setup(self):
        self.start = numpy.array(self.start)
        self.end = numpy.array(self.end)
        self.n = self.time * self.rate
        self.delta = (self.end - self.start) / float(self.n - 1)
        self.i = 1

    def __iter__(self):
        return self

    def next(self):
        if self.i is None:
            self.setup()
        if self.i < self.n:
            i = self.i
            self.i += 1
            return self.start + self.delta * i
        else:
            return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-s', '--start', default="", type=str)
    p.add_argument('-e', '--end', default="1.1,0.0,-0.3", type=str)
    p.add_argument('-t', '--time', default=5., type=float)
    p.add_argument('-r', '--rate', default=10., type=float)
    args = p.parse_args()

    rospy.init_node('stompyleg_trace', anonymous=True)
    rospy.Subscriber(
        "/stompyleg/joint_states", sensor_msgs.msg.JointState, new_joints)

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
    while not rospy.is_shutdown():
        if path.start is None:
            # (read out start position)
            if joints is not None:
                path.start = leg.forward(
                    joints['hip'], joints['thigh'], joints['calf'])
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
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
