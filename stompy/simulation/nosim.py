#!/usr/bin/env python

import os

import numpy

from .. import signaler


global sim
sim = None


class Sim(signaler.Signaler):
    def __init__(self):
        super(Sim, self).__init__()
        self.legs = {}
        self.geometries = {}
        self.build()

    def register_leg(self, leg):
        n = ''.join([s[0].lower() for s in leg.leg_name.split('-')])
        self.geometries[n] = leg.geometry

    def build(self):
        # TODO load terrain
        pass

    def set_joint_angles(self, angles):
        """
        angles: dict, key = leg_number, values = dict
            key = joint name, value = joint angle
            joint names = hip, thigh, knee
        """
        self.legs.update(angles)  # by ref, not copy
        #for ln in angles:
        #    if ln not in self.legs:
        #        self.legs[ln] = {}
        #    for jn in angles[ln]:
        #        self.legs[ln][jn] = angles[ln][jn]

    def update_calves(self):
        # angles -> xyz -> z -> calf load
        for ln in self.legs:
            g = self.geometries[ln]
            l = self.legs[ln]
            pts = list(g.angles_to_points(l['hip'], l['thigh'], l['knee']))
            ft = pts[-1]
            z = ft[2]
            if z < -40:
                cv = -4000  # newtons
            else:
                cv = 0
            self.legs[ln]['calf'] = cv

    def get_joint_states(self):
        """
        states: dict, key = leg_number, values = dict
            key = joint name, value = joint angle
            joint names = hip, thich, knee, calf
        """
        self.update_calves()
        self.trigger('joints', self.legs)
        return self.legs

    def get_orientation(self):
        # TODO simulate this?
        ori = [0., 0., 0.]
        self.trigger('orientation', ori)
        return ori

    def update(self):
        # TODO compute calves periodically
        pass


def get():
    global sim
    if sim is None:
        sim = Sim()
    return sim
