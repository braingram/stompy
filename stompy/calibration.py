#!/usr/bin/env python

import numpy


# TODO store in ~/.stompy/calibrations/<leg>?
setup = {
    1: [  # fl calibration: 180114
        # hip was swapped (min length = max val), swapped lengths
        # 20% symmetric deadband, 100% max, 13 bit
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        #('adc_limits', (0, 6520, 50680)),
        #('adc_limits', (1, 3258, 61003)),
        #('adc_limits', (2, 10701, 60554)),
        ('adc_limits', (0, 5503, 43823)),  # calibration 181111
        ('adc_limits', (1, 3170, 61941)),
        ('adc_limits', (2, 10017, 60332)),
        #('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        #('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        #('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (0, 1.0, 0.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 0.1, 0.0, -8192, 8192)),
        ('pid_config', (2, 1.0, 0.1, 0.0, -8192, 8192)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('calf_scale', (0.000033110771, 0.77719798066)),  # 181125
    ],
    2: [  # 180513 180422
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        #('adc_limits', (0, 8030, 47620)),
        #('adc_limits', (1, 3131, 61003)),  # min/max didn't max out
        #('adc_limits', (2, 7954, 58132)),
        #('adc_limits', (0, 6804, 48278)),
        #('adc_limits', (1, 7111, 65169)),  # spi 4147
        #('adc_limits', (2, 8259, 58023)),
        ('adc_limits', (0, 5370, 49898)),  # 181111 calibration
        ('adc_limits', (1, 3033, 61750)),
        ('adc_limits', (2, 8151, 58062)),
        #('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        #('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        #('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (0, 1.0, 0.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 0.1, 0.0, -8192, 8192)),
        ('pid_config', (2, 1.0, 0.1, 0.0, -8192, 8192)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('calf_scale', (0.000037742337, 0.71074486121)),  # 181125
    ],
    3: [  # 180422
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        #('adc_limits', (0, 4205, 44824)),
        #('adc_limits', (1, 2057, 60996)),  # min/max was able to max thigh
        #('adc_limits', (2, 7789, 59447)),
        ('adc_limits', (0, 4248, 45002)),  # 181112 calibration
        ('adc_limits', (1, 2246, 61017)),
        ('adc_limits', (2, 7879, 58321)),
        #('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        #('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        #('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (0, 1.0, 0.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 0.1, 0.0, -8192, 8192)),
        ('pid_config', (2, 1.0, 0.1, 0.0, -8192, 8192)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('calf_scale', (0.000042698401, 0.646449456328)),  # 181125
    ],
    4: [  # 180422
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        #('adc_limits', (0, 6897, 46052)),
        #('adc_limits', (1, 3063, 61121)),  # spi 4147
        #('adc_limits', (2, 8876, 59055)),
        ('adc_limits', (0, 6646, 46020)),  # 181112 calibration
        ('adc_limits', (1, 2974, 61708)),
        ('adc_limits', (2, 8964, 59123)),
        #('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        #('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        #('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (0, 1.0, 0.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 0.1, 0.0, -8192, 8192)),
        ('pid_config', (2, 1.0, 0.1, 0.0, -8192, 8192)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        #('calf_scale', (-0.000035722324, 2.843468622955)),  # 181125
        ('calf_scale', (-0.000035722324, 3.0738776147350708)),  # 190106
    ],
    5: [  # mr calibration: 170807
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        #('adc_limits', (0, 7072, 52161)),
        #('adc_limits', (1, 3587, 59747)),
        #('adc_limits', (2, 9040, 59069)),
        ('adc_limits', (0, 6874, 52192)),  # 181112 calibration
        ('adc_limits', (1, 3035, 61462)),
        ('adc_limits', (2, 9149, 59182)),
        #('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        #('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        #('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (0, 1.0, 0.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 0.1, 0.0, -8192, 8192)),
        ('pid_config', (2, 1.0, 0.1, 0.0, -8192, 8192)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('calf_scale', (-0.000039140201, 3.14910949125)),  # 181125
    ],
    6: [  # fr calibration: 170904
        # 20% symmetric deadband, 100% max, 13 bit
        ('pwm_limits', (0, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (1, 1638, 8192, 1638, 8192)),
        ('pwm_limits', (2, 1638, 8192, 1638, 8192)),
        #('adc_limits', (0, 6155, 51979)),
        #('adc_limits', (1, 2218, 60632)),
        #('adc_limits', (2, 3417, 54144)),
        ('adc_limits', (0, 6032, 46509)),  # 181112 calibration
        ('adc_limits', (1, 2253, 60809)),
        ('adc_limits', (2, 3391, 53885)),
        #('pid_config', (0, 1.0, 1.0, 0.0, -8192, 8192)),
        #('pid_config', (1, 2.0, 3.0, 0.0, -8192, 8192)),
        #('pid_config', (2, 2.0, 3.0, 0.0, -8192, 8192)),
        ('pid_config', (0, 1.0, 0.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 0.1, 0.0, -8192, 8192)),
        ('pid_config', (2, 1.0, 0.1, 0.0, -8192, 8192)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('dither', (2500, 250)),
        ('calf_scale', (-0.000034554292, 2.771722232637)),  # 181125
    ],
    7: [  # fake leg
        ('pwm_limits', (0, 0, 8192, 0, 8192)),
        ('pwm_limits', (1, 0, 8192, 0, 8192)),
        ('pwm_limits', (2, 0, 8192, 0, 8192)),
        ('pwm_limits', (2, 0, 8192, 0, 8192)),
        ('adc_limits', (0, 100, 65435)),
        ('adc_limits', (1, 100, 65435)),
        ('adc_limits', (2, 100, 65435)),
        ('pid_config', (0, 2.0, 1.0, 0.0, -8192, 8192)),
        ('pid_config', (1, 2.0, 1.0, 0.0, -8192, 8192)),
        ('pid_config', (2, 2.0, 1.0, 0.0, -8192, 8192)),
    ],
}


def generate_calf_calibration(load0, value0, load1, value1, slope=None):
    a = 8.
    b = 18.
    bl = 18.
    in2lb = 600.
    tc = lambda lb: numpy.arccos(
        (a * a + b * b - (bl - lb / in2lb) ** 2.) / (2 * a * b))
    c0 = tc(load0)
    # c0 = v * slope + offset
    if slope is None:  # else only use load0, value0
        c1 = tc(load1)
        slope = (c0 - c1) / (value0 - value1)
    # print(c0, c1)
    # offset = c - v * slope
    offset = c0 - value0 * slope
    return slope, offset
