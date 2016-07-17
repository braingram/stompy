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
import actionlib
import control_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg


import stompy.kinematics.leg as leg
from stompy.planners.trajectory import PathTracer
import stompy.sensors.joints as joints


def path_to_trajectory(
        path, dt, leg_name, delay=0.5):
    msg = control_msgs.msg.FollowJointTrajectoryGoal()
    msg.trajectory.joint_names.append(
        'stompy__body_to_%s' % (leg_name))
    msg.trajectory.joint_names.append(
        'stompy__%s__hip_to_thigh' % (leg_name))
    msg.trajectory.joint_names.append(
        'stompy__%s__thigh_to_calf_upper' % (leg_name))
    # throw out first point
    path.next()
    pt = path.next()
    t = dt
    while pt is not None:
        a = leg.inverse(pt[0], pt[1], pt[2])
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(t)
        p.positions = list(a)
        msg.trajectory.points.append(p)
        t += dt
        pt = path.next()
    msg.trajectory.header.stamp = (
        rospy.Time.now() + rospy.Duration(delay))
    # define tolerances
    msg.goal_tolerance
    msg.goal_time_tolerance = rospy.Duration(1.0)
    return msg


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-s', '--start', default="", type=str)
    p.add_argument('-e', '--end', default="1.8,0.0,0.5", type=str)
    p.add_argument('-t', '--time', default=5., type=float)
    p.add_argument('-r', '--rate', default=10., type=float)
    args = p.parse_args()

    rospy.init_node('stompy_trace', anonymous=True)
    rospy.Subscriber(
        "/stompy/joint_states", sensor_msgs.msg.JointState,
        lambda data, description=joints.stompy_leg_descriptions:
        joints.update_joints(data, description))

    c = actionlib.SimpleActionClient(
        '/stompy/fl/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    c.wait_for_server()

    start = None
    if args.start != "":
        start = map(float, args.start.split(','))
    end = map(float, args.end.split(','))
    pts = [
        (1.8, 0, 0.5),
        (1.2, 1., 0.8),
        (1.4, -1., 0.2),
    ]
    pi = 0
    print("Target: %s" % end)
    sleep_time = 1. / args.rate
    print("Waiting for connection...")
    rospy.sleep(1.)
    state = 0
    while not rospy.is_shutdown():
        if state == 0:
            # (read out start position)
            if joints.joints is not None:
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
                state = 1
        else:  # goal should have been sent
            if c.get_state() == 1:
                pass
            elif c.get_state() == 3:
                print("Done moving")
                pi += 1
                state = 0
                if pi >= len(pts):
                    pi = 0
            else:
                raise Exception("%s" % c.get_state())
        rospy.sleep(sleep_time)

if __name__ == '__main__':
    main()
