#!/usr/bin/env python
"""
Restriction based leg calculator
"""

import numpy


foot_centers = {
    'fr': (1.95627967418, 3.19594053089),
    'mr': (2.3608, 0.0),
    'rr': (1.95627967418, -3.19594053089),
    'fl': (-1.95627967418, 3.19594053089),
    'ml': (-2.3608, 0.0),
    'rl': (-1.95627967418, -3.19594053089),
}
closest_neighbors = {
    'fr': ('fl', 'mr'),
    'mr': ('fr', 'rr'),
    'rr': ('mr', 'rl'),
    'fl': ('fr', 'ml'),
    'ml': ('fl', 'rl'),
    'rl': ('ml', 'rr'),
}

all_neighbors = {
    'fr': ('fl', 'mr', 'ml', 'rr', 'rl'),
    'mr': ('fr', 'rr', 'fl', 'ml', 'rl'),
    'rr': ('mr', 'rl', 'fr', 'fl', 'ml'),
    'fl': ('fr', 'ml', 'mr', 'rl', 'rr'),
    'ml': ('fl', 'rl', 'fr', 'mr', 'rr'),
    'rl': ('ml', 'rr', 'mr', 'fl', 'fr'),
}

neighbors = closest_neighbors
#neighbors = all_neighbors


class Foot(object):
    radius = 1.075
    eps = numpy.log(0.1) / radius
    stance_velocity = 0.2
    swing_velocity = 0.4
    velocity_scale = 1.0

    def __init__(self, name):
        self.name = name
        self.center = foot_centers[name]
        self.position = self.center
        self.state = 'stance'
        self.target = self.position
        #self.stance_velocity = 0.2
        #self.swing_velocity = 0.4
        self.last_update = 0

    def __str__(self):
        return "%s[%s]: %s[%s]" % (
            self.name, self.state, self.position, self.target)

    @property
    def restriction(self):
        cx, cy = self.center
        x, y = self.position
        d = ((cx - x) ** 2. + (cy - y) ** 2.) ** 0.5
        return numpy.exp(-self.eps * (d - self.radius))

    def stance_move(self, dt):
        x, y = self.position
        # target is a direction
        tx, ty = self.target
        self.position = (
            x - tx * self.stance_velocity * dt,
            y - ty * self.stance_velocity * dt)

    def set_state(self, new_state):
        print("%s new state: %s[%s]" % (self.name, self.state, new_state))
        self.state = new_state

    def swing_move(self, dt):
        x, y = self.position
        # target is a position
        tx, ty = self.target
        dx = tx - x
        dy = ty - y
        dl = ((dx * dx) + (dy * dy)) ** 0.5
        ml = dt * self.swing_velocity
        print(self)
        print('\tdist: %s' % dl)
        print('\tdist_target: %s' % ml)
        if dl < ml:  # at target
            self.position = self.target
            self.set_state('wait')
            return
        # move towards target
        self.position = (x + dx / dl * ml, y + dy / dl * ml)

    def update(self, t):
        dt = t - self.last_update
        if self.state == 'swing':
            self.swing_move(dt)
        elif self.state in ('stance', 'wait'):
            self.stance_move(dt)
        self.last_update = t


class RestrictionControl(object):
    def __init__(self, feet=None):
        if feet is None:
            feet = {}
            for foot_name in foot_centers:
                feet[foot_name] = Foot(foot_name)
        self.feet = feet
        self.restrictions = {}
        self.last_lift_times = {}
        for foot in self.feet:
            self.restrictions[foot] = self.feet[foot].restriction
            self.last_lift_times[foot] = 0
        self.restriction_threshold = 0.45
        self.max_restriction = 0.9
        self.step_size = 0.5
        self.max_feet_up = 3

    def lift_foot(self, foot_name, target):
        foot = self.feet[foot_name]
        foot.set_state('swing')
        cx, cy = foot.center
        tx, ty = target
        if (tx == 0 and ty == 0):
            # move to center
            print("Moving %s to center: %s" % (foot.name, foot.center))
            foot.target = (cx, cy)
            return
        tl = ((tx * tx) + (ty * ty)) ** 0.5
        foot.target = (
            cx + tx / tl * self.step_size,
            cy + ty / tl * self.step_size)

    def update(self, t, target):
        states = {}
        restricted = []
        up = []
        down = []
        stance_target = target
        self.previous_restrictions = self.restrictions.copy()
        # first get all foot restrictions
        for foot_name in self.feet:
            self.restrictions[foot_name] = self.feet[foot_name].restriction
        # combine with cross-foot restriction
        # TODO find a/p axis
        #rw = 0.0
        #self.restrictions['fr'] = max(
        #    self.restrictions['fr'], self.restrictions['mr'] * rw)
        #self.restrictions['mr'] = max(
        #    self.restrictions['mr'], self.restrictions['rr'] * rw)
        #self.restrictions['fl'] = max(
        #    self.restrictions['fl'], self.restrictions['ml'] * rw)
        #self.restrictions['ml'] = max(
        #    self.restrictions['ml'], self.restrictions['rl'] * rw)
        # check against max restriction
        max_r = max(self.restrictions.values())
        if max_r > self.max_restriction:
            stance_target = (0, 0)
        elif max_r > self.restriction_threshold:
            # 1.0 at threshold, 0.0 at max
            Foot.velocity_scale = (
                1 - (max_r - self.restriction_threshold)
                / (self.max_restriction - self.restriction_threshold))
            print("Velocity scale:", Foot.velocity_scale)
        # move feet
        for foot_name in self.feet:
            foot = self.feet[foot_name]
            # set target direction if 'down'
            if foot.state in ('wait', 'stance'):
                foot.target = stance_target
            foot.update(t)
            pr = self.previous_restrictions[foot_name]
            r = self.restrictions[foot_name]
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
        for foot in restricted[:]:
            for n in neighbors[foot]:
                if n in up and foot in restricted:
                    print("%s neighbor %s is up" % (foot, n))
                    restricted.remove(foot)
        # if nothing is restricted, return
        if not len(restricted):
            return states
        # if already have max feet up, don't lift any more
        if len(up) >= self.max_feet_up:
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
