#!/usr/bin/env python
"""
Playback movements including:
    - body movements [maybe have these just be leg movements for now]
        for a certain amount of time
        until a position is reached
    - leg movements
        for a certain amount of time
        until a position is reached
    - pause
    - repeat? (should it be possible to define cycles?)

Some examples are:
    - joint excercise: move limbs on pre-set paths
    - from standing, lean back, lift front two legs
    - other...

leg.plan.Plan(
    mode=consts.PLAN_STOP_MODE,
    frame=consts.PLAN_BODY_FRAME,
    linear=...
    angular=...
    matrix=...
    speed=0.)

plans can run for certain amount of time or have other start/stop conditions
I might need conditional transitions for things like:
    - if a leg is loaded do a
    - if a leg is unloaded do b

-- Sequence element --
'plan':
 leg? body?
 kwargs: to build plan

'start': start condition
  needs access to:
   time: since start of sequence
   load: function evaluated with load
   leg xyz, other leg xyz
   previously executed plans on this and other legs
   basically everything

'stop': stop condition, similar to start condition

-- examples --
1) move legs 1 and 6 up then down

p1:l1:move to start location, when there, end p1
p2:l6:move to start location, when there, end p2
p3:l1:when p1 and p2 are done, move up, when at xyz, end p3
p4:l6:when p1 and p2 are done, move up, when at xyz, end p4
p5:l1:when p3 and p4 are done, move down, when at xyz, end p5
p6:l6:when p3 and p4 are done, move down, when at xyz, end p6

p1 lift leg N until z0 (or more)
p2 move to X 70 Y 0
p3 move to X 90 Y 0

2) rear
[all legs]:stop, if loaded, 
"""

import time

import numpy

from . import consts
from . import leg
from . import signaler


# if thrown in an action, causes entire playback to stop
class AbortError(Exception):
    pass


class Action(object):
    next_id = 0

    def __init__(
            self, plan=None, state_id=None, should_start=None, should_stop=None,
            next_action=-1):
        if state_id is None:
            state_id = next_id
            next_id += 1
        self.state_id = state_id
        self.plan = plan
        self.start_time = None
        self.running = False
        if should_start is not None:
            if should_start is True:
                should_start = lambda c, p: True
            elif should_start is False:
                should_start = lambda c, p: False
            self.should_start = should_start
        if should_stop is not None:
            if should_stop is True:
                should_stop = lambda c, p: True
            elif should_stop is False:
                should_stop = lambda c, p: False
            self.should_stop = should_stop
        self.next_action = next_action

    def get_plan(self):
        if self.plan is None:
            p = leg.plans.stop()
        elif isinstance(self.plan, dict):
            p = leg.plans.Plan(**self.plan)
        else:
            p = self.plan
        return p

    def start(self, leg):
        leg.send_plan(self.get_plan())
        self.start_time = time.time()
        self.running = True
        print("Starting plan[%s]: %s" % (self.state_id, self.plan))

    def should_start(self, controller, playback):
        # return true if plan is to be started
        return False

    def should_stop(self, controller, playback):
        # return true if plan is to be stopped
        return True


class Sequence(object):
    def __init__(self, current, actions):
        #self.actions = {}  # actions by ids
        #self.current = {}  # current actions
        self.current = current.copy()
        self.actions = actions.copy()
        self.previous = {}  # previous actions
        for ln in current:
            self.previous[ln] = []
            #self.current[ln] = None
        self.start_time = None
        self.pause_time = None
        self.running = False

    def update(self, controller):
        if self.start_time is None:
            self.start_time = time.time()

        # check deadman, only update when pressed
        if controller.deadman and not self.running:
            # start playing
            self.running = True
            for ln in controller.legs:
                c = self.current.get(ln, None)
                if c is not None and c.running:
                    print("Resuming plan")
                    c.start(controller.legs[ln])
        elif not controller.deadman and self.running:
            # pause
            self.pause_time = time.time()
            self.running = False

        if not self.running:
            return False

        # check if current plans are done
        all_none = True
        for ln in controller.legs:
            c = self.current.get(ln, None)
            #if c is None:  # if plan is None, do nothing
            #    # controller.legs[ln].stop()
            #    pass
            #else:
            if c is not None:
                all_none = False
                if (
                        not c.running and
                        c.should_start(controller, self)):
                    print("Plan not running and should start")
                    c.start(controller.legs[ln])
                elif (
                        c.running and
                        c.should_stop(controller, self)):
                    print("Plan running and should stop")
                    if ln not in self.previous:
                        self.previous[ln] = []
                    self.previous[ln].append(c.state_id)
                    if c.next_action is None:
                        # stop leg
                        controller.legs[ln].stop()
                        self.current[ln] = None
                    else:
                        # leg has next plan
                        c = self.actions[c.next_action]
                        if c.should_start(controller, self):
                            c.start(controller.legs[ln])
                        self.current[ln] = c

        # if all plans are done (all current are None), go to next playback
        return all_none


class Playback(object):
    def __init__(self, sequences):
        self.sequences = sequences
        self.aborted = False

    def update(self, controller):
        if self.aborted or len(self.sequences) == 0:
            return
        try:
            r = self.sequences[0].update(controller)
        except AbortError:
            self.aborted = True
            controller.stop()
            return
        if r:
            self.sequences.pop(0)


def make_leg_wiggle_playback():
    current = {
        6: Action(
            plan=None,
            state_id=0,
            should_start=True,
            should_stop=True,
            next_action=1,
        ),  # stop
    }

    def close_enough(xyz, target, margin):
        # print(xyz, target, margin)
        return abs(
            numpy.linalg.norm(
                numpy.array([xyz['x'], xyz['y'], xyz['z']]) -
                numpy.array(target))) < margin

    def above_z(xyz, z):
        # print(xyz)
        return xyz['z'] > z

    actions = {
        1: Action(  # lift
            plan={
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': [0, 0, 1],
                'speed': 3
            },
            state_id=1,
            should_start=True,
            should_stop=lambda c, p: above_z(c.legs[6].xyz, 0),
            next_action=2,
        ),
        2: Action(  # move to x 70 y 0, z 0
            plan={
                'mode': consts.PLAN_TARGET_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': [70, 0, 0],
                'speed': 3,
            },
            state_id=2,
            should_start=True,
            should_stop=lambda c, p: close_enough(c.legs[6].xyz, [70, 0, 0], 1),
            next_action=3,
        ),
        3: Action(  # move to x 90 y 0 z 0
            plan={
                'mode': consts.PLAN_TARGET_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': [90, 0, 0],
                'speed': 3,
            },
            state_id=3,
            should_start=True,
            should_stop=lambda c, p: close_enough(c.legs[6].xyz, [90, 0, 0], 1),
            next_action=None,
        ),
    }
    return Playback([Sequence(current, actions), ])


def make_rear_playback():
    # state ids are 'namespaced' by leg number * 100
    # leg 1 states all start with 100, etc...
    current = {
        i: Action(
            plan=None,
            state_id=i*100,
            should_start=True,
            should_stop=True,
            next_action=1,
        ) for i in range(1, 7)}

    def close_enough(xyz, target, margin):
        return abs(
            numpy.linalg.norm(
                numpy.array([xyz['x'], xyz['y'], xyz['z']]) -
                numpy.array(target))) < margin

    def above_z(xyz, z):
        return xyz['z'] > z

    # from leg 1 to leg 6
    # - lift leg until loaded less than N
    # - lift N more inches
    # - move to target x, y
    # - lower until loaded more than N
    # level legs?
    # lift body by moving simultaneously down with all legs
    # pitch nose up by N degrees
    # lift front two legs until unloaded
    # lift front two legs N more inches
    # move front two legs around
    # lower front two legs until loaded
    # unpitch body

    actions = {
        1: Action(  # lift
            plan={
                'mode': consts.PLAN_VELOCITY_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': [0, 0, 1],
                'speed': 3
            },
            state_id=1,
            should_start=True,
            should_stop=lambda c, p: above_z(c.legs[6].xyz, 0),
            next_action=2,
        ),
        2: Action(  # move to x 70 y 0, z 0
            plan={
                'mode': consts.PLAN_TARGET_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': [70, 0, 0],
                'speed': 3,
            },
            state_id=2,
            should_start=True,
            should_stop=lambda c, p: close_enough(c.legs[6].xyz, [70, 0, 0], 1),
            next_action=3,
        ),
        3: Action(  # move to x 90 y 0 z 0
            plan={
                'mode': consts.PLAN_TARGET_MODE,
                'frame': consts.PLAN_LEG_FRAME,
                'linear': [90, 0, 0],
                'speed': 3,
            },
            state_id=3,
            should_start=True,
            should_stop=lambda c, p: close_enough(c.legs[6].xyz, [90, 0, 0], 1),
            next_action=None,
        ),
    }
    return Playback(current, actions)


def load_playback(filename, controller):
    fn = os.path.expanduser(filename)
    if not os.path.exists(fn):
        return None
    return Playback(fn, controller)
