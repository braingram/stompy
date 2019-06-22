#!/usr/bin/env python
"""
Store state of all feet + legs so things do not have to register for signals?

Use roll, pitch, yaw and height to compute CG

Use roll, pitch, yaw and foot positions, measure flatness of ground

For support polygon (and 'height') need to know:
    - foot xyz positions in body coordinates
    - foot states (loaded vs unloaded: stance/wait vs lift/lower/swing)

For CG need to know:
    - support triangle (see above)
    - height (see above)
    - roll & pitch to scale projection of CM onto support triangle
"""

import numpy

from . import signaler


class Stance(signaler.Signaler):
    def __init__(self, legs):
        super(Stance, self).__init__()
        self.leg_positions = {}
        self.leg_states = {}
        for leg in legs:
            self.leg_positions[leg] = None
            self.leg_states[leg] = None
        self.height = None
        self.support_polygon = None
        # TODO probably higher up and some inches back
        self.CM = numpy.array([0.0, 0.0, 0.0])

    def on_leg_xyz(self, body_xyz, leg_number):
        # assume xyz is in body coordinates, no reason to know leg coordinates
        self.leg_positions[leg_number] = body_xyz
        # if leg is 'supporting' update support triangle
        if leg_number not in self.leg_states:
            return
        if self.leg_states[leg_number] in ('stance', 'wait'):
            self.update_support_polygon()

    def on_leg_state(self, state, leg_number):
        if leg_number in self.leg_states:
            old_state = self.leg_states[leg_number]
        else:
            old_state = None
        self.leg_states[leg_number] = state
        # if leg is now or was 'supporting' update support triangle
        if state in ('stance', 'wait') or old_state in ('stance', 'wait'):
            self.update_support_polygon()

    def update_support_polygon(self):
        self.support_polygon = []
        support_legs = []
        for leg in self.leg_states:
            if self.leg_states[leg] in ('stance', 'wait'):
                if leg not in self.leg_positions:
                    # not enough data to compute support polygon
                    self.support_polygon = None
                    return
                self.support_polygon.append(self.leg_positions[leg])
                support_legs.append(leg)
        # compute height by taking average of all zs
        self.support_polygon = numpy.array(self.support_polygon)
        self.trigger('support_legs', support_legs)
        print(support_legs)
        self.trigger('support_polygon', self.support_polygon)
        self.height = -numpy.mean(self.support_polygon[:, 2])
        self.trigger('height', self.height)
