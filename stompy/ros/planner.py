#!/usr/bin/env python
"""
Plans are where to go next. They should contain the information needed to:
    - check how close to completion the current plan is
    - append a plan to a current plan at some time
    - cancel a plan
    - compute joint trajectories from the plan:
        - in multiple coordinate frames (joint, foot, body)
        - that follow a target path
        - that have an indefinite endpoint

Example plans are:
    - move leg joints in a direction
    - move a foot in a direction (in foot coordinates)
    - move a foot to a target
"""

import numpy

import rospy

from .. import info
from .. import kinematics
from .. import planners
from . import legs
from .. import sensors
from . import trajectories
from .. import transforms


JOINT_FRAME = 0
FOOT_FRAME = 1
BODY_FRAME = 2

STOP_MODE = 0
TRAJECTORY_MODE = 1
VELOCITY_MODE = 2
TRANSFORM_MODE = 3


def valid_point_positions(pt):
    for p in pt.positions:
        if not numpy.isfinite(p):
            print("=========== invalid point: %s" % pt)
            return False
    return True


class Plan(object):
    def __init__(self, leg):
        self.last_trajectory = None
        self.mode = STOP_MODE
        self.leg = leg
        self._current_point = None
        self.last_publish = None

    def set_stop(self):
        t = rospy.Time.now()
        # find current goal
        pt, ptt = self.current_point()
        if pt is not None and valid_point_positions(pt):
            if ptt > t:
                # TODO send mini trajectory to this point
                print("mini trajectory")
                ps = pt.positions
                trajectory = trajectories.from_angles(
                    self.leg, [ps, ],
                    delay=ptt)
                if valid_point_positions(pt):
                    #print(trajectory)
                    self.send_trajectory(trajectory, JOINT_FRAME)
        else:
            legs.publishers[self.leg].cancel_goal()
        self.mode = STOP_MODE
        #if self.last_trajectory is not None:
        #    legs.publishers[self.leg].cancel_goal()
        #    t = rospy.Time.now()
        #    # find current goal
        #    pt, ptt = self.current_point()
        #    if pt is not None and valid_point_positions(pt):
        #        if ptt > t:
        #            # TODO send mini trajectory to this point
        #            print("mini trajectory")
        #            ps = pt.positions
        #            trajectory = trajectories.from_angles(
        #                self.leg, [ps, ],
        #                delay=ptt)
        #            if valid_point_positions(pt):
        #                #print(trajectory)
        #                self.send_trajectory(trajectory, JOINT_FRAME)
        #        else:
        #            print("no mini trajectory")
        #            print(ptt.to_sec(), t.to_sec())
        #        # save to to avoid compounding errors
        #        self._current_point = pt

    def set_line(self, target, frame, duration, start=None, timestamp=None):
        if start is None:
            start, _ = self.find_start(timestamp, frame)
        pts = planners.trajectory.linear_by_rate(
            start, target, duration, 10.)
        # build trajectory
        trajectory = trajectories.from_angles(self.leg, pts, 0.1, timestamp)
        self.set_trajectory(trajectory, frame)

    def set_arc_velocity(
            self, rotation_point, angles, frame,
            timestamp=None, degrees=False):
        """
        angles are in radians (or degrees) per second
        """
        transform = transforms.rotate_about_point_3d(
            rotation_point[0], rotation_point[1], rotation_point[2],
            angles[0], angles[1], angles[2], degrees=degrees)
        self.set_transform(transform, frame, timestamp=timestamp)

    def set_velocity(self, velocity, frame, timestamp=None):
        if timestamp is None:
            timestamp = rospy.Time.now()
        print("velocity: %s, frame: %s" % (velocity, frame))
        # units are meters per second
        # or radians per second
        self.frame = frame
        self.velocity = velocity
        self.mode = VELOCITY_MODE
        trajectory = self.compute_trajectory(timestamp=timestamp)
        #print("trajectory: %s" % (trajectory, ))
        if (
                (self.last_publish is None) or
                (rospy.Time.now() - self.last_publish > rospy.Duration(0.05))):
            self.send_trajectory(trajectory)

    def set_transform(self, transform, frame, timestamp=None):
        if timestamp is None:
            timestamp = rospy.Time.now()
        self.frame = frame
        self.transform = transform
        self.mode = TRANSFORM_MODE
        trajectory = self.compute_trajectory(timestamp=timestamp)
        if (
                (self.last_publish is None) or
                (rospy.Time.now() - self.last_publish > rospy.Duration(0.05))):
            self.send_trajectory(trajectory)

    def find_start(self, timestamp=None, frame=None, use_end=False):
        if timestamp is None:
            timestamp = rospy.Time.now()
        if use_end:
            npt, nptt = self.end_point()
        else:
            npt, nptt = self.current_point(timestamp)
        if npt is None:
            l = sensors.legs.legs[self.leg]
            npt = [l['hip'], l['thigh'], l['knee']]
            nptt = timestamp
        else:
            npt = npt.positions
        if frame is None:
            frame = self.frame
        if frame == FOOT_FRAME:
            # npt to npt (in foot) + velocity
            npt = kinematics.leg.forward(*npt)
        elif frame == BODY_FRAME:
            # npt to npt (in body) + velocity
            npt = kinematics.body.leg_to_body(
                self.leg, *kinematics.leg.forward(*npt))
        return npt, nptt

    def compute_trajectory(self, timestamp=None, append=False):
        trajectory_start_time = timestamp
        if timestamp is None:
            timestamp = rospy.Time.now()
        if self.mode == STOP_MODE:
            return None
        elif self.mode == TRAJECTORY_MODE:
            return self.trajectory
        elif self.mode in (VELOCITY_MODE, TRANSFORM_MODE):
            start, start_time = self.find_start(
                timestamp, frame=self.frame, use_end=append)
            if append or trajectory_start_time is None:
                trajectory_start_time = start_time
            #if append:
            #    npt, nptt = self.end_point()
            #else:
            #    npt, nptt = self.current_point(timestamp)
            ##print("current point: (%s, %s)" % (npt, nptt))
            #if npt is None:
            #    l = sensors.legs.legs[self.leg]
            #    npt = [l['hip'], l['thigh'], l['knee']]
            #    #print("Using foot position: %s" % (npt, ))
            #    start_time = timestamp
            #else:
            #    # generate 1 second trajectory
            #    npt = npt.positions
            #    start_time = nptt
            #end_time = start_time + rospy.Duration(1.)
            #if self.frame == JOINT_FRAME:
            #    start = npt
            #    # npt to npt + velocity
            #elif self.frame == FOOT_FRAME:
            #    # npt to npt (in foot) + velocity
            #    start = kinematics.leg.forward(*npt)
            #elif self.frame == BODY_FRAME:
            #    # npt to npt (in body) + velocity
            #    start = kinematics.body.leg_to_body(
            #        self.leg, *kinematics.leg.forward(*npt))
            if self.mode == VELOCITY_MODE:
                end = [
                    start[0] + self.velocity[0],
                    start[1] + self.velocity[1],
                    start[2] + self.velocity[2]]
                pts = planners.trajectory.linear_by_n(
                    start, end, 10)
            else:  # transform
                pts = planners.trajectory.follow_transform(
                    start, self.transform, 10)[:-1]
            msg = trajectories.from_angles(
                self.leg, pts, 0.1, trajectory_start_time)
            #print("compute done")
            return msg

    def set_trajectory(self, trajectory, frame):
        # set a trajectory directly
        self.frame = frame
        self.trajectory = trajectory
        self.mode = TRAJECTORY_MODE
        self.send_trajectory(trajectory)

    def send_trajectory(self, trajectory, frame=None):
        if frame is None:
            frame = self.frame
        if frame == FOOT_FRAME:
            points = []
            for pt in trajectory.trajectory.points:
                ps = kinematics.leg.inverse(
                    pt.positions[0], pt.positions[1], pt.positions[2])
                pt.positions = ps
                pt.velocities = []
                points.append(pt)
            trajectory.trajectory.points = points
        elif frame == BODY_FRAME:
            points = []
            for pt in trajectory.trajectory.points:
                ps = kinematics.leg.inverse(
                    *kinematics.body.body_to_leg(
                        self.leg, pt.positions[0],
                        pt.positions[1], pt.positions[2]))
                pt.positions = ps
                pt.velocities = []
                points.append(pt)
            trajectory.trajectory.points = points
        # write over joint names
        trajectory.trajectory.joint_names = [
            '%s_hip' % (self.leg, ),
            '%s_thigh' % (self.leg, ),
            '%s_knee' % (self.leg, )]
        # TODO check limits
        for (i, p) in enumerate(trajectory.trajectory.points):
            if not kinematics.leg.check_limits(p.positions, slop=0.008):
                # TODO make this more better
                print("!!!!Trajectory would send angles out of limits")
                print('%s: %s' % (i, p.positions))
                print('frame: %s' % frame)
                # TODO print which ones is out of range
                return
        legs.publishers[self.leg].send_goal(trajectory)
        if trajectory.trajectory.header.stamp.is_zero():
            #print("timestamping trajectory")
            trajectory.trajectory.header.stamp = rospy.Time.now()
        self.last_publish = rospy.Time.now()
        self.last_trajectory = trajectory
        self._current_point = None
        #print("trajectory sent")

    def update(self, timestamp=None):
        if timestamp is None:
            timestamp = rospy.Time.now()
        if self.mode == STOP_MODE:
            pass  # do nothing
        elif self.mode == TRAJECTORY_MODE:
            # check if trajectory is done?
            pass  # trajectory should have already been sent
        elif self.mode in (VELOCITY_MODE, TRANSFORM_MODE):
            # check to see if the trajectory is close to done
            pt, ptt = self.end_point()
            if pt is None:
                return
            # if so, send a new trajectory
            if (ptt - timestamp) < rospy.Duration(0.5):
                #print("Sending new trajectory")
                trajectory = self.compute_trajectory(append=True)
                self.send_trajectory(trajectory)

    def end_point(self):
        if self.last_trajectory is None:
            return None, None
        t = self.last_trajectory.trajectory
        st = t.header.stamp
        pt = t.points[-1]
        ptt = st + pt.time_from_start
        return pt, ptt

    def previous_point(self, timestamp=None):
        if self.last_trajectory is None:
            return None, None
        if timestamp is None:
            timestamp = rospy.Time.now()
        t = self.last_trajectory.trajectory
        st = t.header.stamp
        prev_pt = None
        prev_ptt = None
        for pt in t.points:
            ptt = st + pt.time_from_start
            if ptt >= timestamp:
                break
            prev_pt = pt
            prev_ptt = ptt
        return prev_pt, prev_ptt

    def next_point(self, timestamp=None):
        if self.last_trajectory is None:
            return None, None
        if timestamp is None:
            timestamp = rospy.Time.now()
        t = self.last_trajectory.trajectory
        st = t.header.stamp
        for pt in t.points:
            ptt = st + pt.time_from_start
            if ptt > timestamp:
                return pt, ptt
        return None, None

    def current_point(self, timestamp=None):
        if self.last_trajectory is None:
            print("No last_trajectory found")
            return None, None
        if timestamp is None:
            timestamp = rospy.Time.now()
        if self._current_point is not None:
            if valid_point_positions(self._current_point):
                print("Using cached current_point")
                return self._current_point, timestamp
            else:
                print("cached current_point is invalid")
                print(self._current_point)
                self._current_point = None
        pt, ptt = self.next_point(timestamp)
        if pt is None:
            print("No next point found")
            pt, ptt = self.previous_point(timestamp)
        if pt is None:
            print("No previous point found")
            print(timestamp)
            t = self.last_trajectory.trajectory
            st = t.header.stamp
            for (i, pt) in enumerate(t.points):
                ptt = st + pt.time_from_start
                print(i, ptt)
        return pt, ptt


global plans
plans = {leg: Plan(leg) for leg in info.legs}


def update(timestamp=None):
    if timestamp is None:
        timestamp = rospy.Time.now()
    for leg in plans:
        plans[leg].update(timestamp)


def set_stop():
    for leg in plans:
        plans[leg].set_stop()
