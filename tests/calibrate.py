#!/usr/bin/env python
"""
"""

import time

import numpy
import pylab

import stompy


leg_number = 6
joint_number = 1
limit_finding_pwm = 0.2
adc_move_threshold = 500
min_move_total = 30000

joint_number_to_name = (
    lambda joint_number: ['hip', 'thigh', 'knee'][joint_number])


def idle(leg, seconds, idle_timeout=0.005, break_check=None):
    if break_check is None:
        break_check = lambda: False
    t0 = time.time()
    while ((time.time() - t0) < seconds) and not break_check():
        leg.update()
        time.sleep(idle_timeout)


def find_limit(leg, joint_number, pwm):
    leg.set_estop(stompy.consts.ESTOP_OFF)

    # connect callback to adc
    d = {}

    def adc_callback(values):
        d['adc'] = values.copy()

    leg.on('adc', adc_callback)

    # wait for first adc reading
    #t0 = time.time()
    idle(leg, 1., break_check=lambda dd=d: ('adc' in dd))
    #while 'adc' not in d and (time.time() - t0 < 1.):
    #    leg.update()
    #    time.sleep(0.005)

    if 'adc' not in d:
        # remove callback
        leg.remove_on('adc', adc_callback)
        leg.set_estop(stompy.consts.ESTOP_SOFT)
        raise Exception("Failed to get initial adc reading")

    # TODO check if sv in a 'reasonable' range

    # start moving at some fixed pwm
    pwms = [0., 0., 0.]
    pwms[joint_number] = pwm
    leg.set_pwm(*pwms)
    joint_name = joint_number_to_name(joint_number)

    moved = False
    total_moved = 0

    # every 1 second check to see that sv has 'moved'
    for _ in range(30):
        last = d['adc'].copy()
        idle(leg, 1.)
        if d['adc']['time'] != last['time']:
            dv = d['adc'][joint_name] - last[joint_name]
            print("Moved %s in 1 second" % dv)
            total_moved += abs(dv)
            if abs(dv) < adc_move_threshold:
                print("Failed to move")
                break
            else:
                moved = True
    else:  # no limit reached
        leg.remove_on('adc', adc_callback)
        leg.set_estop(stompy.consts.ESTOP_SOFT)
        raise Exception("Failed to reach limit at %s pwm" % pwm)

    if moved and total_moved > min_move_total:
        limit = d['adc'][joint_name]
        leg.set_pwm(0., 0., 0.)
    else:
        leg.remove_on('adc', adc_callback)
        leg.set_estop(stompy.consts.ESTOP_SOFT)
        raise Exception("Failed to move at %s pwm" % pwm)

    # remove callback
    print("Found limit %s for joint %s" % (limit, joint_name))
    leg.remove_on('adc', adc_callback)
    leg.set_estop(stompy.consts.ESTOP_SOFT)
    return limit


def find_limits(leg, joint_number):
    pwms = [0.1, 0.2, 0.3, 0.4, 0.5]
    pos_limit = None
    neg_limit = None
    for pwm in pwms:
        # try positive limit
        if pos_limit is None:
            try:
                limit = find_limit(leg, joint_number, pwm)
            except:
                limit = None
            if limit is not None:
                pos_limit = limit
        if neg_limit is None:
            try:
                limit = find_limit(leg, joint_number, -pwm)
            except:
                limit = None
            if limit is not None:
                neg_limit = limit
    print("Positive limit: %s" % pos_limit)
    print("Negative limit: %s" % neg_limit)
    return pos_limit, neg_limit


def measure_velocity(leg, joint_number, pwm, move_limits, sensor_limits):
    leg.set_estop(stompy.consts.ESTOP_OFF)

    # connect callback to adc
    d = {}

    def adc_callback(values):
        d['adc'] = values.copy()

    leg.on('adc', adc_callback)

    idle(leg, 1., break_check=lambda dd=d: ('adc' in dd))

    if 'adc' not in d:
        # remove callback
        leg.remove_on('adc', adc_callback)
        leg.set_estop(stompy.consts.ESTOP_SOFT)
        raise Exception("Failed to get initial adc reading")

    joint_name = joint_number_to_name(joint_number)
    adc_value = d['adc'][joint_name]
    if not (sensor_limits[0] < adc_value < sensor_limits[1]):
        leg.remove_on('adc', adc_callback)
        leg.set_estop(stompy.consts.ESTOP_SOFT)
        raise Exception(
            "ADC value [%s] not in range %s" %
            (adc_value, sensor_limits))

    dist_lower = abs(adc_value - move_limits[0])
    dist_higher = abs(adc_value - move_limits[1])
    if (
            (adc_value < move_limits[0]) or
            (
                (adc_value < move_limits[1]) and
                dist_lower < dist_higher)):
        # closer to lower limit
        in_bounds = [
            lambda v: v < move_limits[1],
            lambda v: v > move_limits[0]]
        pwms = [pwm, -pwm]
        reverse_results = False
    else:  # closer to higher limit
        in_bounds = [
            lambda v: v > move_limits[0],
            lambda v: v < move_limits[1]]
        pwms = [-pwm, pwm]
        reverse_results = True

    velocities = []

    for (pwm, in_bound) in zip(pwms, in_bounds):
        # start moving at some fixed pwm
        pwms = [0., 0., 0.]
        pwms[joint_number] = pwm
        leg.set_pwm(*pwms)
        idle(leg, 0.05)

        dts = []
        dvs = []

        # try to move for N seconds
        # every 0.5 second, measure velocity
        # if a limit is reached (or no movement) exit
        min_move = adc_move_threshold / 50.
        for _ in range(100):
            last = d['adc'].copy()
            idle(leg, 0.05)
            if d['adc']['time'] == last['time']:
                # failed to get adc reading, abort
                leg.remove_on('adc', adc_callback)
                leg.set_estop(stompy.consts.ESTOP_SOFT)
                raise Exception("Failed to update adc")
            dts.append((d['adc']['time'] - last['time']))
            dvs.append((d['adc'][joint_name] - last[joint_name]))
            adc_value = d['adc'][joint_name]
            if not (sensor_limits[0] < adc_value < sensor_limits[1]):
                leg.remove_on('adc', adc_callback)
                leg.set_estop(stompy.consts.ESTOP_SOFT)
                raise Exception("adc went out of range")
            if not in_bound(adc_value):
                print("ADC went out of move range")
                break
            if abs(dvs[-1]) < min_move:
                print("Failed to move")
                break
        leg.set_pwm(0., 0., 0.)
        if len(dts) == 0:
            velocity = 0.
        else:
            print("dts: %s" % (dts, ))
            print("dvs: %s" % (dvs, ))
            velocity = (
                sum([(dv / float(dt)) for (dv, dt) in zip(dvs, dts)])
                / float(len(dvs)))
        velocities.append(velocity)
        idle(leg, 1.0)

    if reverse_results:
        velocities = velocities[::-1]

    # remove callback
    print("Found velocities %s for joint %s" % (velocities, joint_name))
    leg.remove_on('adc', adc_callback)
    leg.set_estop(stompy.consts.ESTOP_SOFT)
    return velocities


if __name__ == '__main__':
    # connect to leg
    legs = stompy.leg.teensy.connect_to_teensies()
    if leg_number not in legs:
        raise Exception()
    leg = legs[leg_number]

    #pl, nl = find_limits(leg, joint_number)

    #pwms = [v * 0.005 for v in range(100)]
    #pwms = [v * 0.005 for v in range(100)]
    pwms = numpy.linspace(0.1, 0.4, 21)
    vmap = {}
    for pwm in pwms:
        try:
            vmap[pwm] = measure_velocity(
                leg, joint_number, pwm,
                [12000, 17000], [9000, 21000])
                #[10000, 15000], [7000, 21000])
        except Exception as e:
            print("Hit error: %s" % e)
            break
    leg.set_estop(stompy.consts.ESTOP_SOFT)
    pwms = numpy.array(sorted(vmap))
    for pwm in pwms:
        print("PWM: %s, VS: %s" % (pwm, vmap[pwm]))
    pvs = numpy.array([vmap[pwm][0] for pwm in pwms])
    nvs = numpy.array([vmap[pwm][1] for pwm in pwms])
    pylab.plot(pwms, pvs, label='p')
    pylab.plot(pwms, -nvs, label='n')
    pylab.legend()
    pylab.show()
