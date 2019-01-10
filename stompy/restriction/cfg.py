#!/usr/bin/env python

from .. import signaler


class RestrictionConfig(signaler.Signaler):
    def __init__(self):
        super(RestrictionConfig, self).__init__()
        self.speeds = {
            'stance': 3.,
            'lift': 3.,
            'lower': 3.,
            'swing': 6.,
            'angular': 0.05}
        self.r_thresh = 0.7
        self.r_max = 0.85
        self.max_feet_up = 0
        self.speed_scalar = 1.
        self.height_slop = 3.
        self.dr_smooth = 0.5
        self.eps = 0.9
        self.lift_height = 8.0
        self.lower_height = -55.0
        self.unloaded_weight = 600.
        self.loaded_weight = 300.
        self.swing_slop = 5.0
        self.step_ratio = 0.6
        self.min_hip_distance = 15.0
        self.target_calf_angle = 0.0
        self.speed_by_restriction = False

    def get_speed(self, mode):
        if mode not in self.speeds:
            raise ValueError("Invalid restriction speed mode: %s" % mode)
        return self.speeds[mode] * self.speed_scalar
