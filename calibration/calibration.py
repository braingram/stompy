#!/usr/bin/env python

cylinders = {
    'hip': {
        'min': 16.0,
        'stroke': 8.0,
    },
    'thigh': {
        'min': 24.0,
        'stroke': 14.0,
    },
    'knee': {
        'min': 20.0,
        'stroke': 12.0,
    },
}

# mr 170807
#joint = 'hip'
#min_value = 420
#min_length = 15.9375
#max_value = 3150
#max_length = 23.6875
joint = 'thigh'
min_value = 757
min_length = 26.125
max_value = 2512
max_length = 33.125
#joint = 'knee'
#min_value = 565
#min_length = 20.0
#max_value = 3301
#max_length = 30.5

adc_res = 12

cylinder = cylinders[joint]
cylinder['max'] = cylinder['min'] + cylinder['stroke']

# convert from adc_res to 16 bit
if adc_res != 16:
    conv = 2. ** 16. / 2. ** adc_res
    min_value *= conv
    max_value *= conv

# generate linear model
# value = y, length = x
slope = (max_value - min_value) / float(max_length - min_length)
intercept = max_value - slope * max_length

print("joint: %s" % joint)
print("slope: %s" % slope)
print("intercept: %s" % intercept)

# calculate lengths for min and max
cmin_value = cylinder['min'] * slope + intercept
cmax_value = cylinder['max'] * slope + intercept
print("Min: %s" % cmin_value)
print("Max: %s" % cmax_value)
