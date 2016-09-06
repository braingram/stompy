#!/usr/bin/env python

import sys
import time

import pygame

import rospy
import sensor_msgs.msg

pygame.init()

w, h = (320, 320)
delay = 100  # ms
z_timeout = 0.3
# left/right, forward/back, ...?
axes = [0, 1, 2, None, None, None]
buttons = [
    pygame.K_1,  # trigger
    pygame.K_2,  # back thumb
    pygame.K_3,  # left thumb
    pygame.K_4,  # right thumb

    pygame.K_q,  # left buttons
    pygame.K_w,
    pygame.K_e,
    pygame.K_d,
    pygame.K_s,
    pygame.K_a,

    pygame.K_y,  # right buttons
    pygame.K_t,
    pygame.K_r,
    pygame.K_f,
    pygame.K_g,
    pygame.K_h,
]

# connect to publisher
rospy.init_node('fake_joystick', anonymous=True)
publisher = rospy.Publisher('/joy', sensor_msgs.msg.Joy)

screen = pygame.display.set_mode((w, h))

# x: [-1,1], y: [-1,1] -> x: [0, w], y: [0, h]
to_screen = lambda x, y: (
    int((x + 1) / 2. * w),
    int((-y + 1) / 2. * h),
)

# x: [0, w], y: [0, h] -> x: [-1,1], y: [-1,1]
from_screen = lambda x, y: (
    float(x) / w * 2. - 1.,
    -(float(y) / h * 2. - 1.),
)

global pos

center = (0, 0)
pos = [center[0], center[1], 0.]
button_states = [0 for _ in buttons]
last_z_time = time.time()


def publish_joystick():
    global pos
    # build message
    msg = sensor_msgs.msg.Joy()
    for a in axes:
        if a is not None:
            msg.axes.append(pos[a])
        else:
            msg.axes.append(0.)
    for bs in button_states:
        msg.buttons.append(bs)
    msg.header.stamp = rospy.Time.now()
    # publish
    publisher.publish(msg)

while True:
    modified = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button in (1, 3):  # left, right
                pos[:2] = from_screen(*event.pos)
                modified = True
            elif event.button == 4:  # wheel up
                pos[2] = 1.
                last_z_time = time.time()
                modified = True
            elif event.button == 5:  # wheel down
                modified = True
                last_z_time = time.time()
                pos[2] = -1.
        elif event.type == pygame.MOUSEMOTION:
            if event.buttons[0] or event.buttons[2]:
                pos[:2] = from_screen(*event.pos)
                modified = True
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # left
                pos[:2] = center
                modified = True
        elif event.type == pygame.KEYDOWN:
            if event.key in buttons:
                button_states[buttons.index(event.key)] = 1
                modified = True
        elif event.type == pygame.KEYUP:
            if event.key in buttons:
                button_states[buttons.index(event.key)] = 0
                modified = True
        else:
            print event
            pass
    if pos[2] != 0. and time.time() - last_z_time > z_timeout:
        pos[2] = 0.
        modified = True
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (255, 255, 255), to_screen(*pos[:2]), 5)
    # publish joystick message
    if modified:
        publish_joystick()
    pygame.display.update()
    pygame.time.delay(30)
