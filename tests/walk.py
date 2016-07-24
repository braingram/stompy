#!/usr/bin/env python

import actionlib
import rospy
import sensor_msgs.msg

import stompy.gaits
import stompy.info
import stompy.ros


dt = 0.1


def stand(ez=1.2):
    print("Standing")
    for leg in stompy.info.legs:
        # move down
        ft = stompy.sensors.joints.legs[leg]['foot']
        end = (ft[0], ft[1], ez)
        msg = stompy.ros.trajectories.line(leg, ft, end)
        stompy.ros.legs.publishers[leg].send_goal(msg)


def is_done(legs=None):
    if legs is None:
        legs = stompy.info.legs
    done = True
    for leg in stompy.info.legs:
        p = stompy.ros.legs.publishers[leg]
        s = p.get_state()
        if s != actionlib.GoalStatus.SUCCEEDED:
            done = False
    return done


def wait(legs=None):
    print("Waiting")
    while not is_done(legs):
        rospy.sleep(0.05)
    print("...done")


def generate_leg_cycles():
    cycles = {}
    cycles['fl'] = stompy.kinematics.body.body_to_leg_array(
        'fl', stompy.gaits.wave.generate_cycle(
            fx=3.5, rx=2.5, y=2.5, ratio=0., dt=dt))
    cycles['ml'] = stompy.kinematics.body.body_to_leg_array(
        'ml', stompy.gaits.wave.generate_cycle(
            fx=0.5, rx=-0.5, y=2.5, ratio=1/3., dt=dt))
    cycles['rl'] = stompy.kinematics.body.body_to_leg_array(
        'rl', stompy.gaits.wave.generate_cycle(
            fx=-2.5, rx=-3.5, y=2.5, ratio=2/3., dt=dt))
    cycles['fr'] = stompy.kinematics.body.body_to_leg_array(
        'fr', stompy.gaits.wave.generate_cycle(
            fx=3.5, rx=2.5, y=-2.5, ratio=2/3., dt=dt))
    cycles['mr'] = stompy.kinematics.body.body_to_leg_array(
        'mr', stompy.gaits.wave.generate_cycle(
            fx=0.5, rx=-0.5, y=-2.5, ratio=0., dt=dt))
    cycles['rr'] = stompy.kinematics.body.body_to_leg_array(
        'rr', stompy.gaits.wave.generate_cycle(
            fx=-2.5, rx=-3.5, y=-2.5, ratio=1/3., dt=dt))
    return cycles


def position_legs(cycles):
    for leg in cycles:
        ft = stompy.sensors.joints.legs[leg]['foot']
        c0 = cycles[leg][0]
        end = (c0[0], c0[1], ft[2])
        print("%s, %s, %s" % (leg, ft, end))
        msg = stompy.ros.trajectories.line(leg, ft, end, delay=0.1)
        stompy.ros.legs.publishers[leg].send_goal(msg)


def lift_legs(cycles):
    for leg in cycles:
        ft = stompy.sensors.joints.legs[leg]['foot']
        end = cycles[leg][0]
        print("%s, %s, %s" % (leg, ft, end))
        msg = stompy.ros.trajectories.line(leg, ft, end, delay=0.1)
        stompy.ros.legs.publishers[leg].send_goal(msg)


def send_cycles(cycles):
    for leg in cycles:
        msg = stompy.ros.trajectories.from_points(
            leg, cycles[leg], dt=dt, delay=0.0)
        stompy.ros.legs.publishers[leg].send_goal(msg)


def main():
    stompy.ros.init.init()
    # wait for joints
    while stompy.sensors.joints.joints is None:
        rospy.sleep(0.05)
    # wait for everything to connect
    rospy.sleep(1.)
    cycles = generate_leg_cycles()
#    stand()
#    wait()
    position_legs(cycles)
    wait()
    stand()
    wait()
    lift_legs(cycles)
    while not rospy.is_shutdown():
        wait()
        send_cycles(cycles)


if __name__ == '__main__':
    main()
