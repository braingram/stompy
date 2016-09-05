#!/usr/bin/env python
"""
"""

from .. import info
from .. import sensors


def position_legs(leg_positions=None):
    if leg_positions is None:
        leg_positions = {}
    # first position unloaded legs
    # sort legs by load
    print("Standing")
    # find all unloaded legs
    for leg in info.legs:
        # move down
        ft = sensors.legs.legs[leg]['foot']
        end = (ft[0], ft[1], ez)
        #msg = stompy.ros.trajectories.line(leg, ft, end)
        #stompy.ros.legs.publishers[leg].send_goal(msg)
