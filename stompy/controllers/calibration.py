#!/usr/bin/env python
"""
Controller to calibrate a leg by monitoring sensor readings
and changing pwm directly (no pid)

Things to measure are:
 (to verify operation of board)
 - sensor direction
 - minimum sensor variation (look for disconnected or noisy sensors)
 (to tune board)
 - deadband (min pwm to see movement)
 - sensor limits (set extra wide, drive to limit)
 - sensor units-per-inch (can be calculated from sensor limits)
 - tune pid
 - calibrate calf??

This will need multiple modes/stages for each above, all should have
an update function that reads/records/processes sensor data,
drives the pwm, and checks for end/error conditions.
"""

from .. import signaler


class CalibrationRoutine(signaler.Signaler):
    def __init__(self):
        super(CalibrationRoutine, self).__init__()

    def update(self, state):
        """
        Just package up leg state and passing to update
        instead of attaching/dettaching to avoid circular ref

        Return 0 if finished, >0 if continue, <0 if error
        """
        return -1
