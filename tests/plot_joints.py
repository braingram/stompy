#!/usr/bin/env python

import sys

import matplotlib.cm
import numpy
import pylab

import rospy
import sensor_msgs.msg


topic = '/stompy/joint_states'

joints = ['fr_thigh', ]
if len(sys.argv) > 1:
    joints = sys.argv[1:]
for j in joints[:]:
    if '_' not in j:
        assert j in ('fr', 'fr', 'mr', 'ml', 'rr', 'rl')
        joints.extend(['%s_%s' % (j, sj) for sj in ('hip', 'thigh', 'knee')])
        joints.remove(j)
n = len(joints)
if n == 1:
    inds = [1, ]
else:
    inds = numpy.arange(float(n)) / (n-1)
colors = matplotlib.cm.jet(inds)

print("Plotting joints: %s" % (joints, ))

global joint_states, first
joint_states = {}
first = True


def callback(data):
    global joint_states, first
    t = data.header.stamp.to_sec()
    for (i, n) in enumerate(data.name):
        if n not in joints:
            continue
        if n not in joint_states:
            joint_states[n] = [[], []]
        joint_states[n][0].append(t)
        joint_states[n][1].append(data.position[i])
    update_plot = False
    for n in joint_states:
        if len(joint_states[n][0]) > 10:
            update_plot = True
            break
    if update_plot:
        if first:
            pylab.ion()
        for n in joint_states:
            i = joints.index(n)
            pylab.plot(
                joint_states[n][0], joint_states[n][1],
                label=n, color=colors[i])
            joint_states[n] = [[], []]
        if first:
            pylab.legend()
            first = False
        pylab.gcf().canvas.draw()


if __name__ == '__main__':
    rospy.init_node('joint_plotter', anonymous=True)
    rospy.Subscriber(
        topic, sensor_msgs.msg.JointState, callback)
    rospy.spin()
