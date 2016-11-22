#!/usr/bin/env python
"""
States
 - lower
 - stance
 - wait
 - lift
 - swing

New joystick should trigger:
    - new swing target
    - new stance target

New position/load should trigger:
    - calc if restricted
"""

import numpy

from .. import info


class Foot(object):
    radius = 1.075
    eps = numpy.log(0.1) / radius

    def __init__(self, name):
        self.name = name
        self.center = info.foot_centers[name]
        self.position = self.center
        self.state = 'stance'
        self.stance_target = self.position
        self.swing_target = self.position
        self.stance_velocity = 0.2
        self.swing_velocity = 0.4
        self.last_update = 0

    def __str__(self):
        return "%s[%s]: %s[%s:%s]" % (
            self.name, self.state, self.position, self.stance_target,
            self.swing_target)

    @property
    def restriction(self):
        cx, cy = self.center
        x, y = self.position
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        return numpy.exp(-self.eps * (d - self.radius))

    def stance_move(self, dt):
        x, y = self.position
        # target is a direction
        tx, ty = self.stance_target
        self.position = (
            x - tx * self.stance_velocity * dt,
            y - ty * self.stance_velocity * dt)

    def set_state(self, new_state):
        print("%s new state: %s[%s]" % (self.name, self.state, new_state))
        self.state = new_state

    def swing_move(self, dt):
        x, y = self.position
        # target is a position
        tx, ty = self.swing_target
        dx = tx - x
        dy = ty - y
        dl = ((dx * dx) + (dy * dy)) ** 0.5
        ml = dt * self.swing_velocity
        #print(self)
        #print('\tdist: %s' % dl)
        #print('\tdist_target: %s' % ml)
        if dl < ml:  # at target
            self.position = self.target
            #self.set_state('wait')
            self.set_state('lift')
            return
        # move towards target
        self.position = (x + dx / dl * ml, y + dy / dl * ml)

    def update(self, t):
        dt = t - self.last_update
        if self.state == 'swing':
            self.swing_move(dt)
        else:
            self.stance_move(dt)
        # TODO check if foot is loaded/unloaded
        # to transition from lower -> stance
        # and from lift -> swing
        self.last_update = t


class RestrictionControl(object):
    def __init__(self, feet=None):
        if feet is None:
            feet = {}
            for foot_name in info.foot_centers:
                feet[foot_name] = Foot(foot_name)
        self.leg_neighbors = info.leg_all_neighbors
        self.n_up_max = 1
        self.feet = feet
        self.restrictions = {}
        self.last_lift_times = {}
        for foot in self.feet:
            self.restrictions[foot] = self.feet[foot].restriction
            self.last_lift_times[foot] = 0
        self.restriction_threshold = 0.25
        self.max_restriction = 0.9
        self.step_size = 0.5

    def enable_tripod(self):
        self.leg_neighbors = info.leg_neighbors
        self.n_up_max = 3

    def enable_crawl(self):
        self.leg_neighbors = info.leg_all_neighbors
        self.n_up_max = 1

    def enable_wave(self):
        self.leg_neighbors = info.leg_neighbors
        self.n_up_max = 2

    def lift_foot(self, foot_name, target):
        foot = self.feet[foot_name]
        foot.set_state('lift')
        #foot.set_state('swing')
        cx, cy = foot.center
        tx, ty = target
        if (tx == 0 and ty == 0):
            # move to center
            print("Moving %s to center: %s" % (foot.name, foot.center))
            foot.swing_target = (cx, cy)
            return
        tl = ((tx * tx) + (ty * ty)) ** 0.5
        foot.swing_target = (
            cx + tx / tl * self.step_size,
            cy + ty / tl * self.step_size)

    def update(self, t, target):
        states = {}
        restricted = []
        up = []
        down = []
        stance_target = target
        # check against max restriction
        if max(self.restrictions.values()) > self.max_restriction:
            stance_target = (0, 0)
        # move feet
        for foot_name in self.feet:
            foot = self.feet[foot_name]
            # set target direction if 'down'
            if foot.state in ('wait', 'stance'):
                foot.stance_target = stance_target
            foot.update(t)
            pr = self.restrictions[foot_name]
            r = foot.restriction
            dr = r - pr
            self.restrictions[foot_name] = r
            # if restriction isn't decreasing, switch from wait to stance
            if foot.state == 'wait' and dr > 0:
                print("\t%s dr = %s" % (foot_name, dr))
                foot.set_state('stance')
            if foot.state == 'swing':
                up.append(foot_name)
            else:
                down.append(foot_name)
            states[foot_name] = {
                'position': foot.position,
                'state': foot.state,
                'r': r,
                'dr': dr,
            }
            if r > self.restriction_threshold and foot.state == 'stance':
                restricted.append(foot_name)
        # don't lift feet whose neighbors are up
        if len(up) > self.n_up_max:
            return states
        for foot in restricted[:]:
            for n in self.neighbors[foot]:
                if n in up and foot in restricted:
                    restricted.remove(foot)
        # if nothing is restricted, return
        if not len(restricted):
            return states
        # sort by most to least restricted
        restricted = sorted(
            restricted, key=lambda foot: self.restrictions[foot],
            reverse=True)
        # find least recently used foot
        max_foot = restricted[0]
        max_foot_last_lift_time = self.last_lift_times[max_foot]
        for foot in restricted[1:]:
            if self.last_lift_times[foot] < max_foot_last_lift_time:
                max_foot = foot
                max_foot_last_lift_time = self.last_lift_times[max_foot]
        # lift max_foot
        self.lift_foot(max_foot, target)
        states[max_foot]['state'] = 'swing'
        return states
