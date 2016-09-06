#!/usr/bin/env python
"""
buttons select mode
buttons
0: trigger
1: back thumb
2: left thumb
3: right thumb
4 5 6 : left buttons
9 8 7
12 11 10 : right buttons
13 14 15

modes:
     0: fl, angles, relative
     1: ml
     2: rl
     3: rr
     4: mr
     5: fr
     6: fl, foot, relative
     7: ml
     8: rl
     9: rr
     10: mr
     11: fr

     32: body translation
     33: body rotations
     34: position legs
     ...

button-mode
    4: fl
    5: ml
    6: rl
    7: rr
    8: mr
    9: fr
    10: body translation
    11: body rotations
    12: position legs

joystick axes pr
"""

import rospy
import sensor_msgs.msg

from .. import callbacker


mode_mapping = {
    4: {-1: 6, 6: 0},
    5: {-1: 7, 7: 1},
    6: {-1: 8, 8: 2},
    7: {-1: 9, 9: 3},
    8: {-1: 10, 10: 4},
    9: {-1: 11, 11: 5},
    10: {-1: 32},
    11: {-1: 33},
    12: {-1: 34},
}

global last_state
last_state = None

global mode
mode = None

global callbacks
callbacks = callbacker.Callbacker()


def new_joystick_state(data):
    global last_state, mode
    last_state = data
    # check buttons to set mode
    if sum(data.buttons) == 1:
        for (i, v) in enumerate(data.buttons):
            if v:
                # update mode based on button input
                if i not in mode_mapping:
                    break
                mapping = mode_mapping[i]
                mode = mapping.get(mode, mapping[-1])
                print("Changed to mode: %s" % mode)
                break
    callbacks(data)


def connect_to_joystick():
    rospy.Subscriber("/joy", sensor_msgs.msg.Joy, new_joystick_state)
