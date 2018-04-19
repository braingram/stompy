#!/usr/bin/env python
"""
Controller to calibrate a leg by monitoring sensor readings
and changing pwm directly (no pid)

Things to measure are:
 (to verify operation of board)
 - sensor direction
 - minimum sensor variation (look for disconnected or noisy sensors)
 (to tune board)
 - deadband (min pwm to see movement): per joint, per direction
   - adc -> pwm
 - sensor limits (set extra wide, drive to limit): per joint
   - adc -> pwm
 - sensor units-per-inch (can be calculated from sensor limits): per joint
   - adc -> pwm
 - tune pid: per joint, per direction?
   - pid -> plan
 - calibrate calf: per leg
   - adc -> plan

This will need multiple modes/stages for each above, all should have
an update function that reads/records/processes sensor data,
drives the pwm, and checks for end/error conditions.
"""

from .. import signaler


class CalibrationSubRoutine(object):
    def __init__(self, joint=None, direction=None):
        self.joint = joint
        self.direction = direction
        self.state = 'done'
        self.result = None

    def set_joint(self, joint):
        self.joint = joint

    def set_direction(self, direction):
        self.direction = direction

    def update(self, leg, value):
        return self.state


class CalibrateDeadband(CalibrationSubRoutine):
    def __init__(self, joint=None, direction=None):
        super(CalibrateDeadband, self).__init__(joint, direction)


class CalibrateSensors(CalibrationSubRoutine):
    def __init__(self, joint=None, direction=None):
        super(CalibrateSensors, self).__init__(joint, direction)
        if self.joint is None:
            self.joint = 'hip'
        if self.direction is None:
            if self.joint == 'knee':
                self.direction = 'extend'
            else:
                self.direction = 'retract'
        self.speed = 0.4
        self.state = 'sampling'
        self.n_samples = 10
        self.threshold = None
        self.samples = []

    def update(self, leg, value):
        if self.state == 'sampling':
            self.samples.append(value[self.joint])
            if len(self.samples) == self.n_samples:
                self.state = 'moving'
                leg.enable_pid(False)
                self.threshold = 2.0 * (max(self.samples) - min(self.samples))
                print(self.state, self.samples, self.threshold)
                self.samples = []
        elif self.state == 'moving':
            # set pwms
            pwms = [0, 0, 0]
            ji = ['hip', 'thigh', 'knee'].index(self.joint)
            d = ['retract', '', 'extend'].index(self.direction) - 1
            pwms[ji] = d * self.speed
            leg.set_pwm(*pwms)
            # keep moving?
            self.samples.append(value[self.joint])
            if len(self.samples) > self.n_samples:
                ptp = max(self.samples) - min(self.samples)
                print(self.state, self.samples, ptp)
                if ptp < self.threshold * 2.0:
                    leg.set_pwm(0., 0., 0.)
                    self.state = 'post_sampling'
                    self.samples = []
                else:
                    self.samples = self.samples[-self.n_samples:]
        elif self.state == 'post_sampling':
            self.samples.append(value[self.joint])
            if len(self.samples) >= self.n_samples:
                self.state = 'done'
                # hip: minimum, set max (different for front/back & mid)
                # thigh: minimum
                # knee: maximum
                self.result = float(sum(self.samples)) / len(self.samples)
                print(self.samples)
                print(self.joint, self.direction, self.result)
        return self.state


class CalibrationRoutine(signaler.Signaler):
    def __init__(self):
        super(CalibrationRoutine, self).__init__()
        self.leg = None
        self.subroutine = None

    def set_subroutine(self, subroutine, joint=None):
        if subroutine == 'sensors':
            self.subroutine = CalibrateSensors(joint=joint)

    def on_adc(self, adc):
        """{hip: thigh: knee: calf: time}"""
        if self.subroutine is not None:
            r = self.subroutine.update(self.leg, adc)
            if r == 'done':
                name = self.subroutine.__class__.__name__
                value = self.subroutine.result
                self.leg.log.info({name: value})
                self.trigger(name, value)

    def attach(self, leg):
        if self.leg is not None:
            self.detach(self.leg)
        # monitor joint
        self.leg = leg
        self.leg.on('adc', self.on_adc)
        if self.leg is None:
            return

    def detach(self):
        if self.leg is None:
            return
        self.leg.remove_on('adc', self.on_adc)
        # stop monitoring joint
        self.leg = None

    def update(self, state):
        """
        Just package up leg state and passing to update
        instead of attaching/dettaching to avoid circular ref

        Return 0 if finished, >0 if continue, <0 if error
        """
        return -1
