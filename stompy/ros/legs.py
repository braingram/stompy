#!/usr/bin/env python

import rospy
import actionlib

import control_msgs.msg

from .. import info


global publishers
publishers = None

global goals
goals = None


def connect_to_leg_publishers(legs=None):
    if legs is None:
        legs = info.legs
    global publishers, goals
    publishers = {}
    goals = {}
    for leg_name in legs:
        publishers[leg_name] = actionlib.SimpleActionClient(
            '/stompy/%s/follow_joint_trajectory' % leg_name,
            control_msgs.msg.FollowJointTrajectoryAction)
        goals[leg_name] = None
    return publishers


def send_goal(leg_name, goal):
    global goals, publishers
    publishers[leg_name].send_goal(goal)
    # timestamp goal
    if goal.trajectory.header.stamp.is_zero():
        goal.trajectory.header.stamp = rospy.Time.now()
    goals[leg_name] = goal


def cancel_goal(leg_name):
    global goals, publishers
    publishers[leg_name].cancel_goal()
    goals[leg_name] = None


def get_done(leg_name=None):
    global publishers
    if leg_name is None:
        return all([get_done(ln) for ln in publishers])
    return publishers[leg_name].get_state() == actionlib.GoalStatus.SUCCEEDED


def wait_till_done(leg_name=None, delay=0.1):
    global publishers
    if leg_name is None:
        return [wait_till_done(ln, delay) for ln in publishers]
    p = publishers[leg_name]
    # TODO use p.wait_for_result?
    while True:
        s = p.get_state()
        if s == actionlib.GoalStatus.SUCCEEDED:
            return True
        if s != actionlib.GoalStatus.ACTIVE:
            return False
        rospy.sleep(delay)


def get_last_point(leg_name, timepoint=None):
    # TODO remove? moved to stompy.ros.planner
    global goals
    goal = goals[leg_name]
    if goal is None:
        return None
    if timepoint is None:
        timepoint = rospy.Time.now()
    t = goal.trajectory
    st = t.header.stamp
    prev_pt = None
    prev_ptt = None
    # look through points, find one was just passed in rostime
    for pt in enumerate(t.points):
        ptt = st + pt.time_from_start
        if ptt >= timepoint:
            break
        prev_pt = pt
        prev_ptt = ptt
    return prev_pt, prev_ptt


def get_next_point(leg_name, timepoint=None):
    global goals
    goal = goals[leg_name]
    if goal is None:
        return None
    # look through points, find one that is next in rostime
    if timepoint is None:
        timepoint = rospy.Time.now()
    t = goal.trajectory
    st = t.header.stamp
    for pt in t.points:
        ptt = st + pt.time_from_start
        if ptt > timepoint:
            return pt.positions, ptt
    return None
