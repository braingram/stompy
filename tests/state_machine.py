#!/usr/bin/env python

import actionlib
import rospy
import smach

import stompy
import stompy.ros
# import stompy.sensors.joints


class Startup(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['ready', ],
            # input_keys=[],
            output_keys=['load', ],)

    def execute(self, userdata):
        # wait for joint data
        while stompy.sensors.joints.joints is None:
            rospy.sleep(0.05)
        rospy.sleep(1.)
        # check if on dollies
        load = 0.
        for leg in stompy.info.legs:
            load += stompy.sensors.legs.legs[leg]['load']
        # return if loaded/not-loaded
        userdata.load = load
        return "ready"


class PositionLegs(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['ready', 'error'],
            input_keys=['load', 'leg_positions', 'leg_loads', 'lift_z'])

    def execute(self, userdata):
        if userdata.load > 4000:
            loaded = True
        else:
            loaded = False
        # positon legs one at a time
        # start with least loaded leg
        leg_loads = {}
        for leg in stompy.info.legs:
            leg_loads[leg] = stompy.sensors.legs.legs[leg]['load']
        legs_by_load = sorted(leg_loads, key=lambda k: leg_loads[k])

        high_z = userdata.lift_z
        for leg in legs_by_load:
            if leg not in userdata.leg_positions[leg]:
                pass
            target_position = userdata.leg_positions[leg]
            target_load = userdata.leg_loads[leg]
            # lift leg (high_z)
            start = stompy.sensors.legs.legs[leg]['foot']
            end = (start[0], start[1], high_z)
            msg = stompy.ros.trajectories.line(leg, start, end, delay=0.1)
            publisher = stompy.ros.legs.publishers[leg]
            publisher.send_goal(msg)
            rospy.sleep(0.5)
            while True:
                state = publisher.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    break
                if state != actionlib.GoalStatus.ACTIVE:
                    print(leg, state)
                    return "error"
                rospy.sleep(0.1)
            # move to position (x/y)
            end = (target_position[0], target_position[1], high_z)
            msg = stompy.ros.trajectories.line(
                leg, stompy.sensors.legs.legs[leg]['foot'],
                end, delay=0.1)
            publisher.send_goal(msg)
            rospy.sleep(0.5)
            while True:
                state = publisher.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    break
                if state != actionlib.GoalStatus.ACTIVE:
                    print(leg, state)
                    return "error"
                rospy.sleep(0.1)
            # lower to position (z or load)
            msg = stompy.ros.trajectories.line(
                leg, stompy.sensors.legs.legs[leg]['foot'],
                target_position, delay=0.1)
            publisher.send_goal(msg)
            rospy.sleep(0.5)
            leg_loaded = False
            while True:
                state = publisher.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    break
                if stompy.sensors.legs.legs[leg]['load'] >= target_load:
                    # cancel action
                    publisher.cancel_goal()
                    leg_loaded = True
                    break
                if state != actionlib.GoalStatus.ACTIVE:
                    print(leg, state)
                    return "error"
                rospy.sleep(0.1)
            dz = 0.5
            while loaded and not leg_loaded:
                lower_target = (
                    target_position[0], target_position[1],
                    target_position[2] + dz)
                msg = stompy.ros.trajectories.line(
                    leg, stompy.sensors.legs.legs[leg]['foot'],
                    lower_target, delay=0.1)
                publisher.send_goal(msg)
                rospy.sleep(0.5)
                while True:
                    state = publisher.get_state()
                    if state == actionlib.GoalStatus.SUCCEEDED:
                        break
                    if stompy.sensors.legs.legs[leg]['load'] >= target_load:
                        # cancel action
                        publisher.cancel_goal()
                        leg_loaded = True
                        break
                    if state != actionlib.GoalStatus.ACTIVE:
                        print(leg, state)
                        return "error"
                    rospy.sleep(0.1)
                dz += 0.5

        return 'ready'


class Stand(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["ready", "error"],
            input_keys=['stand_z', ])

    def execute(self, userdata):
        stand_z = userdata.stand_z
        for leg in stompy.info.legs:
            ft = stompy.sensors.legs.legs[leg]['foot']
            end = (ft[0], ft[1], stand_z)
            msg = stompy.ros.trajectories.line(leg, ft, end)
            stompy.ros.legs.publishers[leg].send_goal(msg)
        done = True
        while not done:
            done = True
            for leg in stompy.info.legs:
                p = stompy.ros.legs.publishers[leg].get_state()
                s = p.get_state()
                if s != actionlib.GoalStatus.SUCCEEDED:
                    done = False
                if s != actionlib.GoalStatus.ACTIVE:
                    print(leg, s)
                    return "error"
        return "ready"


if __name__ == '__main__':
    stompy.ros.init.init()
    # setup state machine
    sm = smach.StateMachine(outcomes=["error", "ready"])
    sm.userdata.stand_z = 1.2
    sm.userdata.lift_z = 0.5
    sm.userdata.leg_positions = {
        'fr': (1.5, 0., 0.5),
        'mr': (1.5, 0., 0.5),
        'rr': (1.5, 0., 0.5),
        'fl': (1.5, 0., 0.5),
        'ml': (1.5, 0., 0.5),
        'rl': (1.5, 0., 0.5),
    }
    leg_load = 5600 / 6.
    sm.userdata.leg_loads = {
        'fr': leg_load, 'mr': leg_load, 'rr': leg_load,
        'fl': leg_load, 'ml': leg_load, 'rl': leg_load,
    }
    with sm:
        smach.StateMachine.add(
            'Startup', Startup(),
            transitions={'ready': 'PositionLegs'})
        smach.StateMachine.add(
            'PositionLegs', PositionLegs(),
            transitions={'ready': 'Stand'})
        smach.StateMachine.add(
            'Stand', Stand())

    outcome = sm.execute()
    print("State machine ended with: %s" % outcome)
