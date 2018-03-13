#!/usr/bin/env python
"""
deadman: one_right [button]
thumb_left_x/y: move leg in X Y
one/two_left [axis]: move leg in Z
arrows up/down: change speed
arrows left/right: change leg (to front left/front right)

X [button]: switch to sensor coord moves
square [button]: switch to restriction
circle [button]: switch to leg coord moves
triangle [button]: move both legs

frame/mode
"""

from .. import consts
from .. import log
from .. import signaler


DEADMAN_KEY = 'one_right'

thumb_mid = 130
thumb_db = 5  # +-
thumb_scale = max(255 - thumb_mid, thumb_mid)


class MultiLeg(signaler.Signaler):
    def __init__(self, legs, joy):
        super(MultiLeg, self).__init__()
        self.legs = legs
        self.leg = None
        self.leg_index = None
        # self.conn = leg_teensy
        self.joy = joy
        self.stopped = True
        # stop all legs
        self.all_legs('set_estop', 1)
        self.move_frame = consts.PLAN_SENSOR_FRAME
        self.speeds = {
            consts.PLAN_SENSOR_FRAME: 1200,
            consts.PLAN_LEG_FRAME: 1.5,
        }
        self.speed_scalar = 1.0
        self.speed_scalar_range = (0.1, 2.0)

    def all_legs(self, cmd, *args):
        log.info({"all_legs": (cmd, args)})
        for leg in self.legs:
            getattr(self.legs[leg], cmd)(*args)

    def set_leg(self, index):
        log.info({"set_leg": index})
        self.leg_index = index
        if self.leg_index is None:
            self.leg = None
        else:
            self.leg = self.legs[index]
        self.trigger('set_leg', index)

    def update(self):
        if self.joy is not None:
            evs = self.joy.update()
        else:
            evs = []
        # value by key name, just keep most recent
        kevs = {}
        for e in evs:
            kevs[e['name']] = e['value']
        self.all_legs('update')
        if DEADMAN_KEY in kevs:
            if kevs[DEADMAN_KEY] and self.stopped:  # pressed
                self.all_legs('set_estop', 0)
                self.all_legs('enable_pid', True)
                self.stopped = False
            elif not kevs[DEADMAN_KEY] and not self.stopped:
                # released, turn estop back on
                self.all_legs('set_estop', 1)
                self.stopped = True
        sdt = None
        if kevs.get('up', False):
            # increase speed
            sdt = 0.05
        if kevs.get('down', False):
            # decrease speed
            sdt = -0.05
        if sdt is not None:
            self.speed_scalar = max(
                self.speed_scalar_range[0],
                min(
                    self.speed_scalar_range[1],
                    self.speed_scalar + sdt))
            log.info({"speed_scalar": self.speed_scalar})
            print("Speed scalar set to: %s" % self.speed_scalar)
        new_frame = None
        if kevs.get('cross', False):
            new_frame = consts.PLAN_SENSOR_FRAME
        if kevs.get('circle', False):
            new_frame = consts.PLAN_LEG_FRAME
        #if (
        #        new_frame is not None and
        #        (new_frame != self.move_frame or self.res.enabled)):
        if (
                new_frame is not None and
                new_frame != self.move_frame):
            self.all_legs('stop')
            self.speed_scalar = 1.
            #self.res.enabled = False
            self.move_frame = new_frame
            #print("New frame: %s" % self.move_frame)
            log.info({"new_frame": new_frame})
        #  other modes...
