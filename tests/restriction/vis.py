#!/usr/bin/env python

import sys

import numpy

from vispy import app

from vispy.scene import SceneCanvas
from vispy.scene.visuals import Ellipse
from vispy.color import Color

foot_radius = 0.1
radius = 1.075

black = Color("#000000")
white = Color("#ecf0f1")
blue = Color("#2980b9")
green = Color("#00ff00")
red = Color("#e74c3c")

w, h = 800, 600

global target
target = (0., 0.)

feet = {
    'fr': {
        'center': (1.95627967418, 3.19594053089),
    },
    'mr': {
        'center': (2.3608, 0.0),
    },
    'rr': {
        'center': (1.95627967418, -3.19594053089),
    },
    'fl': {
        'center': (-1.95627967418, 3.19594053089),
    },
    'ml': {
        'center': (-2.3608, 0.0),
    },
    'rl': {
        'center': (-1.95627967418, -3.19594053089),
    },
}


class Canvas(SceneCanvas):
    def on_mouse_move(self, event):
        # set target direction
        x, y = event.pos
        fx = x * 2. / w - 1.
        fy = y * -2. / h + 1.
        #print(fx, fy)
        global target
        target = (fx, fy)
        #self.print_mouse_event(event, 'Mouse move')

    def print_mouse_event(self, event, what):
        modifiers = ', '.join([key.name for key in event.modifiers])
        print('%s - pos: %r, button: %s, modifiers: %s, delta: %r' %
              (what, event.pos, event.button, modifiers, event.delta))


canvas = Canvas(keys="interactive", title="stompy walk", show=True)
v = canvas.central_widget.add_view()
v.camera = 'panzoom'
v.camera.zoom(10, center=v.camera.center)

# draw leg circles
for foot_name in feet:
    foot = feet[foot_name]
    x, y = foot['center']
    ellipse = Ellipse(
        center=(x, y), radius=(radius, radius), color=black,
        border_width=2, border_color=white, num_segments=100, parent=v.scene)
    x += (numpy.random.random() - 0.5) * 0.5
    y += (numpy.random.random() - 0.5) * 0.5
    ellipse = Ellipse(
        center=(x, y), radius=(foot_radius, foot_radius), color=black,
        border_width=1, border_color=red, num_segments=10, parent=v.scene)
    foot['rmarker'] = ellipse
    ellipse = Ellipse(
        center=(x, y), radius=(foot_radius, foot_radius), color=black,
        border_width=1, border_color=blue, num_segments=10, parent=v.scene)
    foot['marker'] = ellipse


def draw_state(states):
    # draw marker.center and rmarker.center
    # set marker.border_color
    # - white down
    # - blue waiting
    # - green stance
    for foot_name in feet:
        state = states[foot_name]
        foot = feet[foot_name]
        foot['marker'].center = state['position']
        foot['rmarker'].center = state['position']
        foot['rmarker'].radius = foot_radius + state['r'] * 0.5
        color = {
            'wait': blue,
            'stance': green,
            'swing': white,
        }[state['state']]
        foot['marker'].border_color = color


def run(ufunc, dt):
    timer = app.Timer()
    global t
    t = 0

    def update(event):
        global target, t
        state = ufunc(t, target)
        draw_state(state)
        t += dt

    timer.connect(update)
    timer.start(dt)

    if sys.flags.interactive != 1:
        app.run()
