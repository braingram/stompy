#!/usr/bin/env python
"""
Supply this a stance plan in body coordinates
it will produce plans in body coordinates
restriction will be updated with foot coordinates

it will produce 'requests' for plans that will be 'accepted'
"""

import numpy

from .. import consts
from .. import kinematics
from . import leg
from .. import log
from . import odometer
from .. import signaler


parameters = {
    #'speed.stance': 5.,
    #'speed.lift': 6.,
    #'speed.lower': 6.,
    #'speed.swing': 8.,
    #'speed.angular': 0.05,
    'speed_by_restriction': False,
    'r_thresh': 0.4,
    #'r_max': 0.85,
    #'r_max': 0.7,
    'r_max': 0.6,
    'max_feet_up': 1,
    'height_slop': 3.,
    'dr_smooth': 0.5,
    'wait_dr_thresh': 0.01,
    'next_res_thresh': 0.1,

    'limit_eps': 0.3,
    'limit_range': 0.9,
    'limit_inflection_ratio': 0.4,
    'calf_eps': 0.3,
    'calf_inflection_ratio': 0.4,
    'min_hip_eps': 0.15,
    'center_eps': 0.1,
    'center_inflection': 5.,
    'center_radius': 30.,
    #'zero_by_center': False,
    #'zero_on_lower': True,

    #'max_calf_angle': numpy.radians(30),
    'max_calf_angle': 30,
    'target_calf_angle': 10.0,
    'lift_height': 12.0,
    'lower_height': -40.0,
    'set_height_on_mode_select': True,
    'min_lower_height': -70,
    'max_lower_height': -40,
    'unloaded_weight': 600.,
    'loaded_weight': 400.,
    'swing_slop': 5.0,
    'step_ratio': 0.3,
    'min_step_size': 6.0,
    #'step_ratio': 0.2,
    #'min_hip_distance': 35.0,
    'min_hip_buffer': 10.0,
}

parameter_metas = {
    'max_feet_up': {'min': 0, 'max': 3},
}


class BodyTarget(object):
    def __init__(self, rotation_center, speed, dz):
        self.rotation_center = rotation_center
        self.speed = speed
        self.dz = dz

    def __eq__(self, other):
        if other is None:
            return False
        return (
            (self.rotation_center == other.rotation_center) and
            (self.speed == other.speed) and
            (self.dz == other.dz))

    def __repr__(self):
        return (
            "BodyTarget(%r, %r, %r)" %
            (self.rotation_center, self.speed, self.dz))


class Body(signaler.Signaler):
    def __init__(self, legs, param):
        """Takes leg controllers"""
        super(Body, self).__init__()
        self.odo = odometer.Odometer()
        self.logger = log.make_logger('Res-Body')
        self.param = param
        self.param.set_param_from_dictionary('res', parameters)
        [
            self.param.set_meta('res.%s' % (k, ), parameter_metas[k])
            for k in parameter_metas]
        self.legs = legs
        self.feet = {}
        self.halted = False
        self.enabled = False
        self.target = None
        inds = sorted(self.legs)
        self.neighbors = {}
        if len(inds) > 1:
            for (i, n) in enumerate(inds):
                if i == 0:
                    self.neighbors[n] = [
                        inds[len(inds) - 1], inds[i+1]]
                elif i == len(inds) - 1:
                    self.neighbors[n] = [inds[i - 1], inds[0]]
                else:
                    self.neighbors[n] = [inds[i - 1], inds[i + 1]]
        for i in self.legs:
            self.feet[i] = leg.Foot(self.legs[i], self.param)
            self.feet[i].on(
                'restriction', lambda s, ln=i: self.on_restriction(s, ln))
            self.feet[i].on(
                'state', lambda s, ln=i: self.on_foot_state(s, ln))
        print("Feet:", self.feet)
        self.disable()

    def set_halt(self, value):
        self.halted = value
        self.trigger('halt', value)

    def enable(self, foot_states):
        self.logger.debug("enable")
        self.enabled = True
        self.set_halt(False)
        #self.halted = False
        # TODO always reset odometer on enable?
        self.odo.reset()
        # TODO set foot states, target?
        for i in self.feet:
            self.feet[i].reset()

    def offset_foot_centers(self, dx, dy):
        for i in self.feet:
            ldx, ldy, _ = kinematics.body.body_to_leg_rotation(i, dx, dy, 0.)
            # TODO limit to inside limits
            # don't allow -X offset?
            if self.param['limit_center_x_shifts'] and ldx < 0:
                ldx = 0
            self.feet[i].center_offset = (ldx, ldy)

    def calc_stance_speed(self, bxy, mag):
        # scale to pid future time ms
        speed = mag * self.param['speed.foot'] * self.param['speed.scalar'] * consts.PLAN_TICK
        #speed = mag * self.get_mode_speed('stance') * consts.PLAN_TICK
        # find furthest foot
        x, y = bxy
        z = 0.
        mr = None
        for i in self.feet:
            tx, ty, tz = kinematics.body.body_to_leg(i, x, y, z)
            r = tx * tx + ty * ty + tz * tz
            if mr is None or r > mr:
                mr = r
        mr = numpy.sqrt(mr)
        # account for radius sign
        rspeed = speed / mr
        max_rspeed = (
            self.param['speed.foot'] / self.param['arc_speed_radius'] *
            self.param['speed.scalar'])
        #max_rspeed = self.param['res.speed.angular'] * self.param['speed.scalar']
        #if numpy.abs(rspeed) > self.get_mode_speed('angular'):
        if numpy.abs(rspeed) > max_rspeed:
            print("Limiting because of angular speed")
            rspeed = max_rspeed * numpy.sign(rspeed)
        # TODO this should adjust speed on times OTHER than set_target
        if self.param['res.speed_by_restriction']:
            rs = self.get_speed_by_restriction()
        else:
            rs = 1.
        return rspeed * rs

    def set_target(self, target=None, update_swing=True):
        if target is None:
            target = self.target
        if not isinstance(target, BodyTarget):
            raise ValueError("Body.set_target requires BodyTarget")
        self.logger.debug({"set_target": (target, update_swing)})
        if self.halted:
            self.logger.debug("set_target while halted")
            # set new pre_halt target
            self._pre_halt_target = target
            # set stance target to stop
            target = BodyTarget((0., 0.), 0., 0.)
            # only update non-swing
            update_swing = False
        self.target = target
        if target.dz != 0.0:
            # TODO update stand height
            pass
        self.odo.set_target(self.target)
        for i in self.feet:
            self.feet[i].set_target(
                target, update_swing=update_swing)

    def disable(self):
        self.logger.debug("disable")
        # save poses
        #import pickle
        #with open('poses.p', 'wb') as f:
        #    pickle.dump(self.odo.poses, f)
        #with open('path.p', 'wb') as f:
        #    pickle.dump(self.odo.get_path(), f)
        self.enabled = False
        for i in self.feet:
            self.feet[i].set_state(None)

    def halt(self):
        if not self.halted:
            self.logger.debug({
                "halt": {
                    'restriction': {
                        i: self.feet[i].restriction for i in self.feet},
                    'states': {
                        i: self.feet[i].state for i in self.feet},
                    '_pre_halt_target': self.target,
                }})
            self._pre_halt_target = self.target
            self.set_target(BodyTarget((0., 0.), 0., 0.), update_swing=False)
            self.set_halt(True)

    def get_speed_by_restriction(self):
        rmax = max([
            self.feet[i].restriction['r'] for i in self.feet
            if self.feet[i].state not in ('swing', 'lower')])
        return max(0., min(1., 1. - rmax))

    def on_foot_state(self, state, leg_number):
        # TODO update 'support' legs
        pass

    def on_restriction(self, restriction, leg_number):
        if not self.enabled:
            return
        # only update odometer when not estopped
        self.odo.update()
        if (
                self.halted and
                (
                    restriction['r'] < self.param['res.r_max'] or
                    self.feet[leg_number] in ('wait', 'swing', 'lower') or
                    restriction['nr'] < restriction['r'])):
            # unhalt?
            maxed = False
            for i in self.feet:
                # make sure foot is not in swing (or lower?)
                #if self.feet[i].state in ('swing', 'lower', 'wait'):
                if self.feet[i].state in ('swing', 'lower', 'wait'):
                    continue
                r = self.feet[i].restriction
                if r['nr'] < r['r']:  # moving to a less restricted spot
                    continue
                if r['r'] > self.param['res.r_max']:
                    maxed = True
            if not maxed:
                self.logger.debug({
                    "unhalt": {
                        'restriction': {
                            i: self.feet[i].restriction for i in self.feet},
                        'states': {
                            i: self.feet[i].state for i in self.feet},
                        '_pre_halt_target': self._pre_halt_target,
                    }})
                self.set_halt(False)
                self.set_target(self._pre_halt_target, update_swing=True)
                #self.set_target(self._pre_halt_target, update_swing=False)
                return
        if (
                restriction['r'] > self.param['res.r_max'] and
                (not self.halted) and
                (self.feet[leg_number].state not in ('wait', 'swing', 'lower')) and
                restriction['nr'] >= restriction['r']):
            self.halt()
            return
        # TODO scale stance speed by restriction?
        if (
                (restriction['r'] > self.param['res.r_thresh']) and
                self.feet[leg_number].state == 'stance'):
            #if self.halted:
            #    print(
            #        leg_number, self.feet[leg_number].state,
            #        restriction)
            # lift?
            # check n_feet up
            states = {i: self.feet[i].state for i in self.feet}
            n_up = len([
                s for s in states.values() if s not in ('stance', 'wait')])
            # check if neighbors are up
            if len(self.neighbors.get(leg_number, [])) == 0:
                #if self.halted:
                #    print("halted but no neighbors")
                return
            ns = self.neighbors[leg_number]
            n_states = [states[n] for n in ns]
            ns_up = len([s for s in n_states if s not in ('stance', 'wait')])
            # check if any other feet are restricted:
            last_lift_times = {}
            for ln in self.feet:
                if ln == leg_number:
                    last_lift_times[ln] = self.feet[ln].last_lift_time
                    continue
                if states[ln] not in ('stance', 'wait'):
                    continue
                if (
                        self.feet[ln].restriction is not None and
                        self.feet[ln].restriction['r'] >
                        self.param['res.r_thresh']):
                    # found another restricted foot
                    #other_restricted.append(ln)
                    last_lift_times[ln] = self.feet[ln].last_lift_time
            #if self.halted:
            #    print("last_lift_times: %s" % last_lift_times)
            #    print("ns_up: %s, n_up: %s" % (ns_up, n_up))
            #  yes? pick least recently lifted
            if ns_up == 0 and n_up < self.param['res.max_feet_up']:
                n_can_lift = self.param['res.max_feet_up'] - n_up
                #if self.halted:
                #    print("n_can_lift: %s" % n_can_lift)
                #if self.halted:
                #    self.feet[leg_number].set_state('lift')
                if len(last_lift_times) > n_can_lift:
                    # TODO prefer lifting of feet with
                    # restriction_modifier != 0
                    # only allow this foot if it was moved later than
                    # the other restricted feet
                    ln_by_lt = sorted(
                        last_lift_times, key=lambda ln: last_lift_times[ln])
                    #if self.halted:
                    #    print(
                    #        "ln_by_lt: %s[%s]" %
                    #        (ln_by_lt, ln_by_lt[:n_can_lift+1]))
                    if leg_number in ln_by_lt[:n_can_lift+1]:
                        if self.feet[leg_number].should_lift():
                            self.feet[leg_number].set_state('lift')
                else:
                    #if self.halted:
                    #    print("lift %s" % leg_number)
                    # check if should lift based on swing target being
                    # > N in from current position
                    if self.feet[leg_number].should_lift():
                        self.feet[leg_number].set_state('lift')
