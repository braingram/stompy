#!/usr/bin/env python
"""
Much of this is from the StompyLegControl firmware
make sure it is up to date
"""

ESTOP_OFF = 0
ESTOP_SOFT = 1
ESTOP_HARD = 2
ESTOP_HOLD = 3
ESTOP_ON = 2
ESTOP_HEARTBEAT = 2
ESTOP_DEFAULT = 2

HEARTBEAT_TIMEOUT = 1.0
HEARTBEAT_PERIOD = HEARTBEAT_TIMEOUT / 2.

LEG_NAMES_BY_NUMBER = {
    0: 'Undefined',
    1: 'Front-Left',
    2: 'Middle-Left',
    3: 'Rear-Left',
    4: 'Rear-Right',
    5: 'Middle-Right',
    6: 'Front-Right',
    7: 'Fake',
}
