#!/usr/bin/env python

import warnings

has_ros = False
try:
    import rospy
    has_ros = True
except ImportError as e:
    warnings.warn("rospy failed to import[%s], skipping ros code" % e)

if has_ros:
    from . import init
    from . import joints
    #from . import joystick
    from . import legs
    from . import trajectories
    #__all__ = ['init', 'joints', 'joystick', 'legs', 'trajectories']
    __all__ = ['init', 'joints', 'legs', 'trajectories']
