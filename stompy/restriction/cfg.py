#!/usr/bin/env python

import numpy

from .. import signaler


class RestrictionConfig(signaler.Signaler):
    def __init__(self):
        super(RestrictionConfig, self).__init__()
        self.speeds = {
            'stance': 8.,
            'lift': 8.,
            'lower': 8.,
            'swing': 12.,
            'angular': 0.05}
        self.r_thresh = 0.3
        self.r_max = 0.85
        self.max_feet_up = 0
        self.speed_scalar = 1.
        self.height_slop = 3.
        self.dr_smooth = 0.5
        self.eps = 1.0
        self.calf_eps = 3.0
        self.max_calf_angle = numpy.radians(30)
        self.lift_height = 8.0
        self.lower_height = -40.0
        self.set_height_on_mode_select = True
        self.min_lower_height = -70
        self.max_lower_height = -40
        self.unloaded_weight = 600.
        self.loaded_weight = 400.
        self.swing_slop = 5.0
        self.step_ratio = 0.4
        self.min_hip_distance = 25.0
        self.target_calf_angle = 0.0
        self.speed_by_restriction = False

    def get_speed(self, mode):
        if mode not in self.speeds:
            raise ValueError("Invalid restriction speed mode: %s" % mode)
        return self.speeds[mode] * self.speed_scalar
