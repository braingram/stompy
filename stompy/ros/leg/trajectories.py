#!/usr/bin/env python

import rospy


global current
current = None


def new_trajectory(trajectory):
    global current
    if current is None:
        current = trajectory
    else:
        current = merge(current, trajectory)


def merge(a, b):
    """Marge a with b where b is a newer trajectory

    This assumes that both trajectories have
    """
    if b.header.stamp.is_zero():
        b.header.stamp = rospy.Time.now()
    # TODO check that these should be merged
    # check that joint names agree
    if a.trajectory.joint_names != b.trajectory.joint_names:
        raise Exception(
            "Joint names do not agree[%s != %s], cannot merge" % (
                a.trajectory.joint_names, b.trajectory.joint_names))
    # TODO merge points
    # copy over tolerances
    a.goal_time_tolerance = b.goal_time_tolerance
    a.path_tolerance = b.path_tolerance
    a.goal_tolerance = b.goal_tolerance
    return a
