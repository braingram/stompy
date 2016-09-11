#!/usr/bin/env python

import numpy
import pylab

import control_msgs.msg
import rospy


leg = "fr"
topic = "/stompy/%s/follow_joint_trajectory/goal" % leg


def callback(data):
    t = data.goal.trajectory
    st = t.header.stamp
    points = []
    for pt in t.points:
        ptt = st + pt.time_from_start
        points.append((
            ptt.to_sec(), pt.positions[0], pt.positions[1], pt.positions[2]))
    points = numpy.array(points)
    pylab.ion()
    nt = rospy.Time.now().to_sec()
    st = st.to_sec()
    pylab.title("black=now[%0.4f], red=start[%0.4f]" % (nt, st))
    pylab.plot(points[:, 0], points[:, 1])
    pylab.axvline(nt, color='k')
    pylab.axvline(st, color='r')
    pylab.gcf().canvas.draw()


if __name__ == '__main__':
    rospy.init_node("goal_plotter", anonymous=True)
    rospy.Subscriber(
        topic, control_msgs.msg.FollowJointTrajectoryActionGoal, callback)
    rospy.spin()
