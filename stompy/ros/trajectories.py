#!/usr/bin/env python
"""
"""


import rospy
import control_msgs.msg
import trajectory_msgs.msg

from .. import kinematics
from .. import planners


default_distance = 0.01  # meters
default_dt = 0.05
default_delay = 0.01


def timestamp(msg, delay=default_delay):
    msg.trajectory.header.stamp = (
        rospy.Time.now() + rospy.Duration(delay))
    return msg


def from_points(leg_name, points, dt=default_dt, delay=default_delay):
    msg = control_msgs.msg.FollowJointTrajectoryGoal()
    msg.trajectory.joint_names.append(
        'stompy__body_to_%s' % (leg_name))
    msg.trajectory.joint_names.append(
        'stompy__%s__hip_to_thigh' % (leg_name))
    msg.trajectory.joint_names.append(
        'stompy__%s__thigh_to_calf_upper' % (leg_name))
    # skip first point
    t = dt
    for pt in points:
        a = kinematics.leg.inverse(pt[0], pt[1], pt[2])
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(t)
        p.positions = list(a)
        msg.trajectory.points.append(p)
        t += dt
    return timestamp(msg, delay)


def line(
        leg_name, start, end,
        distance=default_distance, dt=default_dt, delay=default_delay):
    """
    tolerances aren't defined
    """
    pts = planners.trajectory.linear_by_distance(start, end, distance)
    return from_points(leg_name, pts[1:], dt, delay)


def follow_transform(
        leg_name, start, transform,
        n, dt=default_dt, delay=default_delay):
    """transform is applied every step"""
    pts = planners.trajectory.follow_transform(start, transform, n)
    return from_points(pts[1:])
