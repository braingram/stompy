#!/usr/bin/env python

import heartbeat.heart

global beat
beat = None


def connect():
    global beat
    beat = heartbeat.heart.ServerHeart('head')
