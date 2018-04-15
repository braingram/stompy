#!/usr/bin/env python

# TODO store in ~/.stompy/calibrations/<leg>?
setup = {
    1: [  # fl calibration: 180114
        # hip was swapped (min length = max val), swapped lengths
        # 20% symmetric deadband, 100% max, 13 bit
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        ('adc_limits', (0, 6520, 50680)),
        ('adc_limits', (1, 3258, 61003)),
        ('adc_limits', (2, 10701, 60554)),
        ('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        # calf scale
    ],
    5: [  # mr calibration: 170807
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        ('adc_limits', (0, 7072, 52161)),
        ('adc_limits', (1, 3587, 59747)),
        ('adc_limits', (2, 9040, 59069)),
        ('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
    ],
    6: [  # fr calibration: 170904
        # 20% symmetric deadband, 100% max, 13 bit
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        ('adc_limits', (0, 6155, 51979)),
        ('adc_limits', (1, 2218, 60632)),
        ('adc_limits', (2, 3417, 54144)),
        ('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        # calf scale
    ],
    7: [  # fake leg
        ('pwm_limits', (0, 0, 8192, 0, 8192)),
        ('pwm_limits', (1, 0, 8192, 0, 8192)),
        ('pwm_limits', (2, 0, 8192, 0, 8192)),
        ('pwm_limits', (2, 0, 8192, 0, 8192)),
        ('adc_limits', (0, 100, 65435)),
        ('adc_limits', (1, 100, 65435)),
        ('adc_limits', (2, 100, 65435)),
        ('pid_config', (0, 4.0, 2.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 4.0, 2.0, 0.0, -8192, 8192)),
        ('pid_config', (2, 4.0, 2.0, 0.0, -8192, 8192)),
        # calf scale
    ],
}
