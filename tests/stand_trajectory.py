#!/usr/bin/env python
"""
read in joints
find foot positions
move all feet down/up until body is at z
"""

import argparse

import numpy

import rospy
import actionlib
import control_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg


import stompy.kinematics.leg as leg
import stompy.kinematics.body as body
from stompy.planners.trajectory import PathTracer
import stompy.sensors.joints as joints
from stompy import transforms


def path_to_trajectory(
        path, leg_name, start_time):
    #msg = trajectory_msgs.msg.JointTrajectory()
    #msg = goal.action_goal.goal
    msg = control_msgs.msg.FollowJointTrajectoryGoal()
    msg.trajectory.joint_names.append(
        'stompy__body_to_%s' % (leg_name, ))
    msg.trajectory.joint_names.append(
        'stompy__%s__hip_to_thigh' % (leg_name, ))
    msg.trajectory.joint_names.append(
        'stompy__%s__thigh_to_calf_upper' % (leg_name, ))
    # throw out first point, as the start is the current position here
    path.next()
    pt = path.next()
    total_time = path.time
    dt = total_time / (path.n - 1)
    t = dt
    while pt is not None:
        a = leg.inverse(pt[0], pt[1], pt[2])
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(t)
        p.positions = list(a)
        msg.trajectory.points.append(p)
        t += dt
        pt = path.next()
    # give last point time to settle
    #msg.trajectory.points[-1].time_from_start += rospy.Duration(0.5)
    msg.trajectory.header.stamp = start_time
    #msg.header.stamp = rospy.Time.now() + rospy.Duration(delay)
    for n in (
            'stompy__body_to_%s' % (leg_name, ),
            'stompy__%s__hip_to_thigh' % (leg_name, ),
            'stompy__%s__thigh_to_calf_upper' % (leg_name, )):
        t = control_msgs.msg.JointTolerance()
        t.name = n
        t.position = 0.1
        msg.goal_tolerance.append(t)

    msg.goal_time_tolerance = rospy.Duration(1.0)
    return msg  # goal


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-z', '--sz', default=0.5, type=float)
    p.add_argument('-Z', '--ez', default=1.2, type=float)
    p.add_argument('-x', '--x', default=1.5, type=float)
    p.add_argument('-t', '--time', default=1., type=float)
    p.add_argument('-r', '--rate', default=10., type=float)
    args = p.parse_args()

    # subscribe to joint messages
    rospy.init_node('stompy_stand', anonymous=True)
    rospy.Subscriber(
        "/stompy/joint_states", sensor_msgs.msg.JointState,
        lambda data, description=joints.stompy_leg_descriptions:
        joints.update_joints(data, description))

    # setup publishers for commanding joints
    leg_publishers = {}
    foot_paths = {}
    #for leg_name in ('fl', 'fr', 'ml', 'mr', 'rl', 'rr'):
    for leg_name in ('fl', 'fr', 'ml', 'mr', 'rl', 'rr'):
        #leg_publishers[leg_name] = rospy.Publisher(
        #    lc(leg_name), trajectory_msgs.msg.JointTrajectory, queue_size=10)
        leg_publishers[leg_name] = actionlib.SimpleActionClient(
            '/stompy/%s/follow_joint_trajectory' % leg_name,
            control_msgs.msg.FollowJointTrajectoryAction)
        foot_paths[leg_name] = PathTracer(time=args.time, rate=args.rate)
        foot_paths[leg_name].skip = False

    leg_publishers.values()[0].wait_for_server()

    state = 0  # awaiting joints
    tx = 0.25
    ty = 0.25
    tz = 0.25
    ax = 15
    ay = 15
    az = 15
    moves = [
        # stand
        (2, args.sz, 0),
        (0, args.x, 0),
        (2, args.ez, 0),
        # leg lifts
        (0, args.sz, 4),
        (0, args.ez, 4),
        (1, args.sz, 4),
        (1, args.ez, 4),
        (2, args.sz, 4),
        (2, args.ez, 4),
        (3, args.sz, 4),
        (3, args.ez, 4),
        (4, args.sz, 4),
        (4, args.ez, 4),
        (5, args.sz, 4),
        (5, args.ez, 4),
        # body translations
        (0, tx, 3),
        (0, -tx*2, 3),
        (0, tx, 3),
        (1, ty, 3),
        (1, -ty*2, 3),
        (1, ty, 3),
        (2, tz, 3),
        (2, -tz*2, 3),
        (2, tz, 3),
        # body rotations
        (0, ax, 5),
        (0, -ax*2, 5),
        (0, ax, 5),
        (1, ay, 5),
        (1, -ay*2, 5),
        (1, ay, 5),
        (2, az, 5),
        (2, -az*2, 5),
        (2, az, 5),
    ]
    sleep_time = 1. / args.rate
    print("Waiting for connection...")
    rospy.sleep(1.)
    while not rospy.is_shutdown():
        if state == 0:  # awaiting joints
            if joints.joints is not None:
                # setup paths using computed start points
                target_axis, target, new_state = moves.pop(0)
                if new_state != state:
                    state = new_state
                    moves.insert(0, (target_axis, target, new_state))
                    continue
                for leg_name in joints.legs:
                    # compute foot position
                    foot = leg.forward(
                        joints.legs[leg_name]['hip'],
                        joints.legs[leg_name]['thigh'],
                        joints.legs[leg_name]['knee'])
                    # setup path for foot
                    foot_paths[leg_name].start = foot
                    ep = [foot[0], 0., foot[2]]
                    ep[target_axis] = target
                    h, t, k = leg.inverse(*ep)
                    if not any(
                            (
                                leg.in_limits(h, leg.hip_limits),
                                leg.in_limits(t, leg.thigh_limits),
                                leg.in_limits(k, leg.knee_limits),
                            )):
                        raise Exception()
                    foot_paths[leg_name].end = ep
                    # compute distance, set time
                    distance = numpy.linalg.norm(
                        numpy.array(ep) -
                        numpy.array(foot_paths[leg_name].start))
                    # limit speed to 0.01 m/s
                    t = max(1., distance / 0.75)
                    foot_paths[leg_name].time = t
                    print(
                        "Preparing %s foot move from %s to %s" %
                        (leg_name, foot, ep))
                state = 1  # moving feet
            else:
                print("Awaiting joints")
        elif state == 1:  # moving feet
            start_time = rospy.Time.now() + rospy.Duration(0.5)
            # convert paths to trajectories
            for leg_name in foot_paths:
                if foot_paths[leg_name].skip:
                    continue
                t = path_to_trajectory(
                    foot_paths[leg_name], leg_name,
                    start_time)
                leg_publishers[leg_name].send_goal(t)
                leg_publishers[leg_name].last_goal = t
            state = 2  # go to 'wait' state
        elif state == 2:  # wait
            dones = []
            for leg_name in foot_paths:
                if foot_paths[leg_name].skip:
                    continue
                p = leg_publishers[leg_name]
                s = p.get_state()
                dones.append(s == 3)
                if s == 3:
                    foot_paths[leg_name] = PathTracer(
                        time=args.time, rate=args.rate)
                    foot_paths[leg_name].skip = False
                elif s == 1:
                    pass
                    #ns = p.action_client.ns
                    #lp = p.last_goal.trajectory.points[-1]
                    #cp = numpy.array([
                    #    joints.legs[leg_name][j] for j in
                    #    ('hip', 'thigh', 'calf')])
                    #print leg_name, numpy.array(lp.positions) - cp
                elif s != 1:
                    print s, p.get_result()
                    t = rospy.Time.now()
                    ns = p.action_client.ns
                    lp = p.last_goal.trajectory.points[-1]
                    tt = (
                        lp.time_from_start +
                        p.last_goal.trajectory.header.stamp)
                    leg_name = ns.split('/')[2]
                    ap = joints.legs[leg_name].copy()
                    print(leg_name)
                    print(tt)
                    print(lp)
                    print(ap)
                    raise Exception
            if all(dones):
                print("Done moving")
                if len(moves):
                    state = 0
                else:
                    break
        elif state == 3:  # body shift
            target_axis, target, new_state = moves.pop(0)
            if new_state != state:
                state = new_state
                moves.insert(0, (target_axis, target, new_state))
                continue
            for leg_name in joints.legs:
                # compute foot position
                foot = leg.forward(
                    joints.legs[leg_name]['hip'],
                    joints.legs[leg_name]['thigh'],
                    joints.legs[leg_name]['knee'])
                # setup path for foot
                foot_paths[leg_name].start = foot
                ep = list(body.leg_to_body(leg_name, *foot))
                ep[target_axis] += target
                ep = body.body_to_leg(leg_name, *ep)
                h, t, k = leg.inverse(*ep)
                if not any(
                        (
                            leg.in_limits(h, leg.hip_limits),
                            leg.in_limits(t, leg.thigh_limits),
                            leg.in_limits(k, leg.knee_limits),
                        )):
                    raise Exception()
                foot_paths[leg_name].end = ep
                # compute distance, set time
                #distance = numpy.linalg.norm(
                #    numpy.array(ep) -
                #    numpy.array(foot_paths[leg_name].start))
                ## limit speed to 0.01 m/s
                #t = max(1., distance / 0.1)
                foot_paths[leg_name].time = args.time
                print(
                    "Preparing %s foot move from %s to %s" %
                    (leg_name, foot, ep))
            state = 1
        elif state == 4:
            target_axis, target, new_state = moves.pop(0)
            if new_state != state:
                state = new_state
                moves.insert(0, (target_axis, target, new_state))
                continue
            for (i, leg_name) in enumerate(
                    ('fl', 'fr', 'ml', 'mr', 'rl', 'rr')):
                if i != target_axis:
                    foot_paths[leg_name].skip = True
                    continue
                foot_paths[leg_name].skip = False

                # compute foot position
                foot = leg.forward(
                    joints.legs[leg_name]['hip'],
                    joints.legs[leg_name]['thigh'],
                    joints.legs[leg_name]['knee'])
                # setup path for foot
                foot_paths[leg_name].start = foot
                ep = [foot[0], foot[1], target]
                h, t, k = leg.inverse(*ep)
                if not any(
                        (
                            leg.in_limits(h, leg.hip_limits),
                            leg.in_limits(t, leg.thigh_limits),
                            leg.in_limits(k, leg.knee_limits),
                        )):
                    raise Exception()
                foot_paths[leg_name].end = ep
                # compute distance, set time
                distance = numpy.linalg.norm(
                    numpy.array(ep) -
                    numpy.array(foot_paths[leg_name].start))
                # limit speed to 0.01 m/s
                t = max(1., distance / 0.75)
                foot_paths[leg_name].time = t
                print(
                    "Preparing %s foot move from %s to %s" %
                    (leg_name, foot, ep))
            state = 1
        elif state == 5:  # body rotations
            target_axis, target, new_state = moves.pop(0)
            if new_state != state:
                state = new_state
                moves.insert(0, (target_axis, target, new_state))
                continue
            angles = [0, 0, 0]
            angles[target_axis] = target
            rm = transforms.rotation_matrix_3d(
                angles[0], angles[1], angles[2], degrees=True)
            for leg_name in joints.legs:
                # compute foot position
                foot = leg.forward(
                    joints.legs[leg_name]['hip'],
                    joints.legs[leg_name]['thigh'],
                    joints.legs[leg_name]['knee'])
                # setup path for foot
                foot_paths[leg_name].start = foot
                ep = list(body.leg_to_body(leg_name, *foot))
                # TODO make these not 'straight' lines
                ep = transforms.transform_3d(ep[0], ep[1], ep[2], rm)
                ep = body.body_to_leg(leg_name, *ep)
                h, t, k = leg.inverse(*ep)
                if not any(
                        (
                            leg.in_limits(h, leg.hip_limits),
                            leg.in_limits(t, leg.thigh_limits),
                            leg.in_limits(k, leg.knee_limits),
                        )):
                    raise Exception()
                foot_paths[leg_name].end = ep
                # compute distance, set time
                #distance = numpy.linalg.norm(
                #    numpy.array(ep) -
                #    numpy.array(foot_paths[leg_name].start))
                ## limit speed to 0.01 m/s
                #t = max(1., distance / 0.1)
                foot_paths[leg_name].time = args.time
                print(
                    "Preparing %s foot move from %s to %s" %
                    (leg_name, foot, ep))
            state = 1
        rospy.sleep(sleep_time)


if __name__ == '__main__':
    main()
