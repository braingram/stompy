#!/usr/bin/env python
"""
Take in a Point Message (x, y, z)
move the absolutet from where it is now to that

x, y, z, time?

Point seems better for targets
Vector3 seems better for delta

For now, setup relative movements (Vector3)
In addition to the delta, I need:
    - which leg to move
    - angles or joint coordinates
Maybe this can all be part of the 'mode'
Mode (uint64?)
     0 : fl, angles, relative
     1 : ml, angles, relative
     2 : rl, angles, relative
     3 : fr, angles, relative
     4 : mr, angles, relative
     5 : rr, angles, relative
     6 : fl, angles, absolute
     7 : ml, angles, absolute
     8 : rl, angles, absolute
     9 : fr, angles, absolute
    10 : mr, angles, absolute
    11 : rr, angles, absolute
    12 : fl, foot, relative
    13 : ml, foot, relative
    14 : rl, foot, relative
    15 : fr, foot, relative
    16 : mr, foot, relative
    17 : rr, foot, relative
    18 : fl, foot, absolute
    19 : ml, foot, absolute
    20 : rl, foot, absolute
    21 : fr, foot, absolute
    22 : mr, foot, absolute
    23 : rr, foot, absolute

    32 : body (6 dof), relative
    33 : body (6 dof), absolute

    64 : crawl
    65 : wave gait
    66 : tripod
    67 : omnidirectional

It's probably best to have the goal and mode in the same message.
That way we can avoid errors where mode switching occurs at the wrong time

So the message should look like:
    Header (time) [start time]
    duration [when it should be reached, relative to start time]
    Twist
    Mode (Uint64)
    ...? Extra buttons, flags, etc?
"""

import rospy
import sensor_msgs.msg

import stompy.ros

global last_time
last_time = 0


def new_joystick_state(data):
    print data
    # define end point by axis
    ft = stompy.sensors.joints.legs['fl']['foot']
    x, y, z = data.axes[0], data.axes[1], data.axes[2]
    end = (ft[0] + x, ft[1] + y, ft[2] + z)
    msg = stompy.ros.trajectories.line(
        'fl',
        ft,
        end)
    stompy.ros.legs.publishers['fl'].send_goal(msg)


def main():
    stompy.ros.init.init()
    rospy.Subscriber(
        "/joy", sensor_msgs.msg.Joy, new_joystick_state)
    # wait for startup, wait for initial joints
    while stompy.sensors.joints.joints is None:
        rospy.sleep(0.05)
    rospy.spin()


if __name__ == '__main__':
    main()
