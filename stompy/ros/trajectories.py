#!/usr/bin/env python
"""
                start = leg.forward(
                    joints.legs['fl']['hip'],
                    joints.legs['fl']['thigh'],
                    joints.legs['fl']['knee'])
                path = PathTracer(
                    start=start, end=pts[pi], time=args.time, rate=args.rate)
                print("Found starting position: %s" % (path.start, ))
                print("Move to: %s" % (pts[pi], ))
                g = path_to_trajectory(path, sleep_time, 'fl', 0.1)
                #print("Publish: %s" % g)
                c.send_goal(g)
"""


import rospy
import control_msgs.msg
import trajectory_msgs.msg

from .. import kinematics
from .. import planners


default_distance = 0.01  # meters
default_dt = 0.01
default_delay = 0.1


def line(
        leg_name, start, end,
        distance=default_distance, dt=default_dt, delay=default_delay):
    """
    tolerances are not defined
    """
    pts = planners.trajectory.linear_by_distance(start, end, distance)
    msg = control_msgs.msg.FollowJointTrajectoryGoal()
    msg.trajectory.joint_names.append(
        'stompy__body_to_%s' % (leg_name))
    msg.trajectory.joint_names.append(
        'stompy__%s__hip_to_thigh' % (leg_name))
    msg.trajectory.joint_names.append(
        'stompy__%s__thigh_to_calf_upper' % (leg_name))
    # skip first point
    t = dt
    for pt in pts[1:]:
        a = kinematics.leg.inverse(pt[0], pt[1], pt[2])
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(t)
        p.positions = list(a)
        msg.trajectory.points.append(p)
        t += dt
    msg.trajectory.header.stamp = (
        rospy.Time.now() + rospy.Duration(delay))
    return msg
