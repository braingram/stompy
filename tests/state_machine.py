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
            self, outcomes=['ready', ])
            # input_keys=[],
            #output_keys=['load', ],)

    def execute(self, userdata):
        # wait for joint data
        while stompy.sensors.joints.joints is None:
            rospy.sleep(0.05)
        rospy.sleep(1.)
        ## check if on dollies
        #load = 0.
        #for leg in stompy.info.legs:
        #    load += stompy.sensors.legs.legs[leg]['load']
        ## return if loaded/not-loaded
        #userdata.load = load
        # select action by mode
        while stompy.ros.joystick.mode is None:
            print("Waiting on joystick input...")
            rospy.sleep(0.5)
        return "ready"


class ModeTransition(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=[
                'moveleg', 'movebody', 'positionlegs', 'error'])

    def execute(self, userdata):
        mode = stompy.ros.joystick.mode
        if mode < 12:
            return 'moveleg'
        if mode in (32, 33):
            return 'movebody'
        if mode == 34:
            return 'positionlegs'
        return 'error'


class MoveLeg(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['newmode', 'error'])

    def execute(self, userdata):

        legs_by_mode = ('rl', 'ml', 'fl', 'fr', 'mr', 'rr')

        def move_leg(data, state={}):
            mode = stompy.ros.joystick.mode
            # determine which leg to move
            if mode < 6:  # angles
                leg = legs_by_mode[mode]
                move_type = 'angle'
            elif mode < 12:  # foot
                leg = legs_by_mode[mode - 6]
                move_type = 'foot'
            else:
                return
            # check if a previous message had been sent to a different leg
            if 'leg' in state and state['leg'] != leg:
                # if so, cancel it as the leg changed
                stompy.ros.legs.publishers[state['leg']].cancel_goal()
                del state['leg']
            movement = False
            for a in data.axes:
                if abs(a) > 0.01:
                    movement = True
                    break
            if not movement:
                # TODO check goal before canceling?
                stompy.ros.legs.publishers[leg].cancel_goal()
                if 'leg' in state:
                    del state['leg']
            else:
                # append new trajectory to old?
                if move_type == 'foot':
                    ft = stompy.sensors.legs.legs[leg]['foot']
                    end = (
                        ft[0] + data.axes[0],
                        ft[1] + data.axes[1],
                        ft[2] + data.axes[2])
                    #pts = stompy.planners.trajectory.linear_by_distance(
                    #    ft, end, 0.01)
                    msg = stompy.ros.trajectories.from_points(
                        leg, [end, ], delay=0.0, dt=1.0)
                    #msg = stompy.ros.trajectories.line(
                    #    leg, ft, end)
                else:  # angles
                    leg_angles = stompy.sensors.legs.legs[leg]
                    end = (
                        leg_angles['hip'] + data.axes[0],
                        leg_angles['thigh'] + data.axes[1],
                        leg_angles['knee'] + data.axes[2])
                    msg = stompy.ros.trajectories.from_angles(
                        leg, [end, ], delay=0.0, dt=1.0)
                stompy.ros.legs.publishers[leg].send_goal(msg)
                state['leg'] = leg

        # attach callback to joystick
        cbid = stompy.ros.joystick.callbacks.register(
            move_leg)
        # sleep and watch for new modes
        while True:
            rospy.sleep(0.1)
            if not (stompy.ros.joystick.mode < 12):
                break
        # unregister callback
        stompy.ros.joystick.callbacks.unregister(cbid)
        # transition to new mode
        return "newmode"


class MoveBody(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['newmode', 'error'])

    def execute(self, userdata):

        def move_body(data, state={}):
            mode = stompy.ros.joystick.mode
            if mode not in (32, 33):
                return
            movement = False
            for a in data.axes:
                if abs(a) > 0.01:
                    movement = True
                    break
            if not movement:
                for leg in stompy.info.legs:
                    # TODO check goal before canceling?
                    stompy.ros.legs.publishers[leg].cancel_goal()
                return
            leg_msgs = {}
            if mode == 32:  # translate
                for leg in stompy.info.legs:
                    ft = stompy.sensors.legs.legs[leg]['foot']
                    bft = stompy.kinematics.body.leg_to_body(leg, *ft)
                    bend = (
                        bft[0] + data.axes[0],
                        bft[1] + data.axes[1],
                        bft[2] + data.axes[2])
                    lend = stompy.kinematics.body.body_to_leg(leg, *bend)
                    msg = stompy.ros.trajectories.from_points(
                        leg, [lend, ], delay=0., dt=5.0)
                    #pts = stompy.planners.trajectory.linear_by_distance(
                    #    ft, stompy.kinematics.body.body_to_leg(leg, *bend),
                    #    0.01)
                    #msg = stompy.ros.trajectories.from_points(
                    #    leg, pts[1:], delay=0.1)
                    #msg = stompy.ros.trajectories.line(
                    #    leg, ft,
                    #    stompy.kinematics.body.body_to_leg(leg, *bend),
                    #    delay=0.1)
                    leg_msgs[leg] = msg
            else:  # 33, rotate
                rm = stompy.transforms.rotation_3d(
                    data.axes[0], data.axes[1], data.axes[2], degrees=True)
                for leg in stompy.info.legs:
                    ft = stompy.sensors.legs.legs[leg]['foot']
                    bft = stompy.kinematics.body.leg_to_body(leg, *ft)
                    # follow transform
                    bpts = stompy.planners.trajectory.follow_transform(
                        bft, rm, 10)
                    pts = stompy.kinematics.body.body_to_leg_array(leg, bpts)
                    #msg = stompy.ros.trajectories.from_points(
                    #    leg, pts[1:], delay=0.1)
                    msg = stompy.ros.trajectories.from_points(
                        leg, [pts[-1], ], delay=0.0, dt=5.0)
                    leg_msgs[leg] = msg
            # send trajectories
            for leg in leg_msgs:
                stompy.ros.legs.publishers[leg].send_goal(leg_msgs[leg])

        # attach callback to joystick
        cbid = stompy.ros.joystick.callbacks.register(
            move_body)
        # sleep and watch for new modes
        while True:
            rospy.sleep(0.1)
            if not (stompy.ros.joystick.mode in (32, 33)):
                break
        # unregister callback
        stompy.ros.joystick.callbacks.unregister(cbid)
        # transition to new mode
        return "newmode"


class PositionLegs(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['ready', 'error'],
            input_keys=['leg_positions', 'leg_loads', 'lift_z'])

    def execute(self, userdata):
        load = 0.
        for leg in stompy.info.legs:
            load += stompy.sensors.legs.legs[leg]['load']
        if load > 4000:
            loaded = True
            pt_distance = 0.01
        else:
            loaded = False
            pt_distance = 0.04
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
            msg = stompy.ros.trajectories.line(
                leg, start, end, distance=pt_distance, delay=0.0)
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
                end, distance=pt_distance, delay=0.0)
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
                target_position, distance=pt_distance, delay=0.0)
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
                    lower_target, delay=0.0)
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


class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["error", "newmode"])

    def execute(self, userdata):
        while True:
            if stompy.ros.joystick.mode != 34:
                return "newmode"
            rospy.sleep(0.1)
        return "error"


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
            transitions={'ready': 'ModeTransition'})
        smach.StateMachine.add(
            'ModeTransition', ModeTransition(),
            transitions={
                'moveleg': 'MoveLeg', 'movebody': 'MoveBody',
                'positionlegs': 'PositionLegs'})
        smach.StateMachine.add(
            'MoveLeg', MoveLeg(),
            transitions={'newmode': 'ModeTransition'})
        smach.StateMachine.add(
            'MoveBody', MoveBody(),
            transitions={'newmode': 'ModeTransition'})
        smach.StateMachine.add(
            'PositionLegs', PositionLegs(),
            transitions={'ready': 'Stand'})
        smach.StateMachine.add(
            'Stand', Stand(),
            transitions={'ready': 'Wait'})
        smach.StateMachine.add(
            'Wait', Wait(),
            transitions={'newmode': 'ModeTransition'})

    outcome = sm.execute()
    print("State machine ended with: %s" % outcome)
