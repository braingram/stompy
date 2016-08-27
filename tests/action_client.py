#!/usr/bin/env python

import actionlib
import control_msgs.msg
import rospy
import trajectory_msgs.msg

rospy.init_node('action_client', anonymous=True)

c = actionlib.SimpleActionClient(
    '/action_server', control_msgs.msg.FollowJointTrajectoryAction)

if not c.wait_for_server():
    raise Exception("Failed to find server")

g = control_msgs.msg.FollowJointTrajectoryGoal()
g.trajectory.joint_names = ['a', 'b']


def make_point(a, b, t):
    pt = trajectory_msgs.msg.JointTrajectoryPoint()
    pt.positions = [a, b]
    pt.time_from_start = rospy.Time(t)
    return pt

g.trajectory.points = [make_point(i, i * 2, i / 10) for i in xrange(1, 10)]
g.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
c.send_goal(g)

# wait for result
if not c.wait_for_result():
    raise Exception("waiting for result failed")

r = c.get_result()
# result == control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
if r != 0:  # successful
    raise Exception("goal was not successful")

# c.get_statate() == actionlib.GoalStatus.PREEMPTED
