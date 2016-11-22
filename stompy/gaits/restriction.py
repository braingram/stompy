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
        self.state = 'stance'
        self.last_lift_time = None

    def __str__(self):
        return "%s[%s]" % (
            self.name, self.state)

    def compute_restriction(self, position):
        """position is x, y in body coordinates"""
        x, y = position
        cx, cy = self.center
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        return numpy.exp(-self.eps * (d - self.radius))

    def set_state(self, new_state):
        print("%s new state: %s[%s]" % (self.name, self.state, new_state))
        # TODO trigger new plan
        self.state = new_state


class RestrictionControl(object):
    def __init__(self):
        self.feet = {}
        for foot_name in info.foot_centers:
            foot = Foot(foot_name)
            foot.last_restriction = None
            foot.restriction = None
            self.feet[foot_name] = foot
        self.n_up_max = 1  # set to 3 for tripod, 1 for crawl
        self.restriction_threshold = 0.05
        self.max_restriction = 0.95

    def compute_foot_restrictions(self, foot_positions):
        max_r = 0
        for foot_name in self.feet:
            foot = self.feet[foot_name]
            foot.last_restriction = foot.restriction
            foot.restriction = foot.compute_restriction(
                foot_positions[foot_name])
            max_r = max(max_r, foot.restriction)
            # print(foot_name, foot.restriction)
        return max_r

    def update(self, t, foot_positions):
        request_states = {}
        restricted = []
        up = []
        down = []
        # compute foot restrictions
        max_r = self.compute_foot_restrictions(foot_positions)
        if max_r > self.max_restriction:
            # TODO override all stance_targets, pause for swings
            return {f: 'pause' for f in self.feet}
        for foot_name in self.feet:
            foot = self.feet[foot_name]
            if foot.state == 'wait':
                dr = foot.restriction - foot.last_restriction
                if dr > 0:
                    request_states[foot_name] = 'stance'
                    foot.set_state('stance')
            if foot.state == 'swing':
                up.append(foot_name)
            elif foot.state in ('stance', 'wait'):
                down.append(foot_name)
            if (
                    foot.restriction > self.restriction_threshold and
                    foot.state == 'stance'):
                restricted.append(foot_name)
        #print("Up feet: %s" % (up, ))
        #print("Down feet: %s" % (down, ))
        #print("Restrictions: %s" % (
        #    [(f, self.feet[f].restriction) for f in self.feet]))
        # don't lift feet whose neighbors are up
        if len(self.feet) - len(down) >= self.n_up_max:
            return request_states
        for foot in restricted[:]:
            for n in info.leg_neighbors[foot]:
                if n not in down and foot in restricted:
                    restricted.remove(foot)
        # if nothing is restricted, return
        if not len(restricted):
            return request_states
        # sort by most to least restricted
        restricted = sorted(
            restricted, key=lambda foot: self.feet[foot].restriction,
            reverse=True)
        # find least recently used foot
        max_foot = restricted[0]
        max_foot_last_lift_time = self.feet[max_foot].last_lift_time
        for foot in restricted[1:]:
            if self.feet[foot].last_lift_time < max_foot_last_lift_time:
                max_foot = foot
                max_foot_last_lift_time = self.feet[max_foot].last_lift_time
        # lift max_foot
        request_states[max_foot] = 'swing'
        #self.lift_foot(max_foot, target)
        return request_states
