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
import trajectory_msgs.msg


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

    fl = rospy.Publisher(
        '/stompyleg/fl/command',
        trajectory_msgs.msg.JointTrajectory)

    start = None
    if args.start != "":
        start = map(float, args.start.split(','))
    end = map(float, args.end.split(','))
    path = PathTracer(start=start, end=end, time=args.time, rate=args.rate)
    pt = None
    print("Target: %s" % end)
    sleep_time = 1. / args.rate
    print("Waiting for connection...")
    rospy.sleep(1.)
    while not rospy.is_shutdown():
        if path.start is None:
            # (read out start position)
            if joints.joints is not None:
                path.start = leg.forward(
                    joints.legs['fl']['hip'],
                    joints.legs['fl']['thigh'],
                    joints.legs['fl']['calf'])
                print("Found starting position: %s" % (path.start, ))
                angles = []
                pt = path.next()
                while pt is not None:
                    a = leg.inverse(pt[0], pt[1], pt[2])
                    angles.append(a)
                    pt = path.next()
                print("Move angles: %s" % angles)
                # publish all points as a trajectory message
                msg = trajectory_msgs.msg.JointTrajectory()
                #msg.header.seq = n
                #n += 1
                msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
                msg.joint_names.append('stompyleg__body_to_fl')
                msg.joint_names.append('stompyleg__fl__hip_to_thigh')
                msg.joint_names.append('stompyleg__fl__thigh_to_calf_upper')
                #msg.joint_names = [
                #    'stompyleg__body_to_fl'
                ##    'stompyleg__fl__hip_to_thigh'
                #    'stompyleg__fl__thigh_to_calf_upper'
                #]
                t = sleep_time
                for a in angles[1:]:  # skip the first angle as it's the start
                    p = trajectory_msgs.msg.JointTrajectoryPoint()
                    p.time_from_start = rospy.Duration(t)
                    p.positions = list(a)
                    msg.points.append(p)
                    t += sleep_time
                msg.points
                # publish the trajectory
                print("Publish: %s" % msg)
                fl.publish(msg)
        rospy.sleep(sleep_time)

if __name__ == '__main__':
    main()
