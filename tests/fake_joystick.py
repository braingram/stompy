#!/usr/bin/env python

import sys
import pygame

import rospy
import sensor_msgs.msg

pygame.init()

w, h = (320, 320)
delay = 100  # ms
# left/right, forward/back, ...?
axes = [0, 1, None, None, None, None]
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

center = (0, 0)
pos = center
button_states = [0 for _ in buttons]


def publish_joystick():
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
                pos = from_screen(*event.pos)
                modified = True
        elif event.type == pygame.MOUSEMOTION:
            if event.buttons[0] or event.buttons[2]:
                pos = from_screen(*event.pos)
                modified = True
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # left
                pos = center
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
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (255, 255, 255), to_screen(*pos), 5)
    # publish joystick message
    if modified:
        publish_joystick()
    pygame.display.update()
    pygame.time.delay(30)
