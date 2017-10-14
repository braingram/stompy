#!/usr/bin/env python
"""
"""

import Tkinter as tk

import calc


foot_radius = 0.1

#black = Color("#000000")
#white = Color("#ecf0f1")
#blue = Color("#2980b9")
#green = Color("#00ff00")
#red = Color("#e74c3c")

pw, ph = 600, 600
cw, ch = 9, 9

tr = 5  # target radius
fr = 5  # foot radius
fc = {
    'wait': 'blue',
    'stance': 'green',
    'swing': 'white',
}

global target
target = (0., 0.)


class App(object):
    def __init__(self, rc, root=None):
        self._sliders = {}
        self.rc = rc
        self.feet = {}
        for foot in calc.foot_centers:
            self.feet[foot] = {
                'center': calc.foot_centers[foot],
            }

        if root is None:
            root = tk.Tk()
        self.root = root
        self.target = (0., 0.)
        self.t = 0.0
        self.dt = 0.0

        # add window & canvas
        self.pw, self.ph = pw, ph
        self.cw, self.ch = cw, ch
        self.canvas = tk.Canvas(
            self.root, width=self.pw, height=self.ph,
            bg='black')
        self.canvas.pack(side=tk.LEFT)

        #Foot:
        #    stance_velocity: 0.2
        #    swing_velocity: 0.4
        #RestrictionControl:
        #    restriction threshold: 0.25
        #    step_size: 0.5
        """
        f = tk.Frame(self.root)
        l = tk.Label(f, text="stance v")
        l.pack(side=tk.TOP)
        self.v = tk.DoubleVar()
        self.v.set(calc.Foot.stance_velocity)

        def set_stance_velocity(a, b, c, v=self.v):
            calc.Foot.stance_velocity = v.get()

        s = tk.Scale(f, from_=0, to=1, resolution=0.05, variable=self.v)
        self.v.trace("w", set_stance_velocity)
        s.pack(side=tk.TOP)
        f.pack(side=tk.LEFT)
        """
        self.make_slider(
            "stace v", calc.Foot.stance_velocity,
            lambda v: setattr(calc.Foot, "stance_velocity", v))
        self.make_slider(
            "swing v", calc.Foot.swing_velocity,
            lambda v: setattr(calc.Foot, "swing_velocity", v))
        self.make_slider(
            "r thresh", self.rc.restriction_threshold,
            lambda v: setattr(self.rc, "restriction_threshold", v))
        self.make_slider(
            "step", self.rc.step_size,
            lambda v: setattr(self.rc, "step_size", v))
        self.make_slider(
            "max up", self.rc.max_feet_up,
            lambda v: setattr(self.rc, "max_feet_up", v),
            from_=1, to=3, resolution=1)

        # initialize canvas
        # target: white rect
        tx, ty = self._c2p(*self.target)
        self.target_rect = self.canvas.create_rectangle(
            tx - tr, ty - tr, tx + tr, ty + tr,
            fill='white')
        for foot in self.feet:
            foot = self.feet[foot]
            # feet: white circles
            fx, fy = self._c2p(*foot['center'])
            foot['marker'] = self.canvas.create_oval(
                fx - fr, fy - fr, fx + fr, fy + fr,
                outline='white')
            # restriction: red circles
            foot['rmarker'] = self.canvas.create_oval(
                fx - fr, fy - fr, fx + fr, fy + fr,
                outline='red')
            foot['rtext'] = self.canvas.create_text(
                fx + fr, fy + fr,
                fill='red',
                text='?',
                anchor='nw')
            # travel limits: yellow circles
            r = calc.Foot.radius * min(pw, ph) / float(max(cw, ch))
            foot['lmarker'] = self.canvas.create_oval(
                fx - r, fy - r, fx + r, fy + r,
                outline='yellow')

        # monitor mouse movement
        self.canvas.bind("<B1-Motion>", self.set_target)
        self.canvas.bind("<Button-1>", self.set_target)
        self.canvas.bind("<Button-3>", self.reset_target)

    def make_slider(self, name, value, set_value, **kwargs):
        f = tk.Frame(self.root)
        l = tk.Label(f, text=name)
        l.pack(side=tk.TOP)
        v = tk.DoubleVar()
        v.set(value)

        def func(a, b, c, v=v):
            print("setting to %s" % v.get())
            set_value(v.get())

        kw = {'from_': 0, 'to': 1, 'resolution': 0.05, 'variable': v}
        kw.update(kwargs)
        #s = tk.Scale(f, from_=0, to=1, resolution=0.05, variable=v)
        s = tk.Scale(f, **kw)
        v.trace("w", func)
        s.pack(side=tk.TOP)
        f.pack(side=tk.LEFT)
        self._sliders[name] = {'var': v}

    def _p2c(self, x, y):
        return (
            ((x * 2.0 / self.pw) - 1.) * (self.cw / 2.),
            ((y * -2.0 / self.ph) + 1.) * (self.ch / 2.))

    def _c2p(self, x, y):
        return (
            ((x / (self.cw / 2.) + 1.) * self.pw / 2.),
            ((y / (self.ch / 2.) - 1.) * self.ph / -2.))

    def set_target(self, event):
        tx, ty = self._p2c(event.x, event.y)
        # update target
        self.target = (tx, ty)

    def reset_target(self, event):
        self.target = (0., 0.)

    def redraw(self, states):
        # draw target: white rectangle
        tx, ty = self._c2p(*self.target)
        self.canvas.coords(
            self.target_rect,
            (tx - tr, ty - tr, tx + tr, ty + tr))

        for foot in self.feet:
            state = states[foot]
            foot = self.feet[foot]
            fx, fy = self._c2p(*state['position'])
            s = state['state']
            r = state['r']
            # draw feet
            self.canvas.coords(
                foot['marker'],
                (fx - fr, fy - fr, fx + fr, fy + fr))
            # draw restriction
            rr = fr + r * 20
            self.canvas.coords(
                foot['rmarker'],
                (fx - rr, fy - rr, fx + rr, fy + rr))
            self.canvas.coords(foot['rtext'], (fx + fr, fy + fr))
            self.canvas.itemconfig(
                foot['rtext'],
                text='%0.2f[%s]' % (r, s),
                fill=fc[s])
            # draw state
            self.canvas.itemconfig(
                foot['marker'],
                outline=fc[s])

    def update(self):
        states = self.ufunc(self.t, self.target)
        self.redraw(states)
        self.t += self.dt
        # call after dt
        self.root.after(int(self.dt * 1000), self.update)

    def run(self, ufunc, dt):
        self.ufunc = ufunc
        self.dt = dt
        self.root.after(int(self.dt * 1000), self.update)
        self.root.mainloop()


def run(rc, ufunc, dt):
    app = App(rc)
    app.run(ufunc, dt)
