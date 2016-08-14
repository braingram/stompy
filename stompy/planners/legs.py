#!/usr/bin/env python
"""
States
 - lower
 - stance
 - lift
 - swing
phase: maybe dynamically controlled
"""


class Leg(object):
    def __init__(self, name):
        self.name = name
