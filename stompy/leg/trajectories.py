#!/usr/bin/env python

global current
current = None


def new_trajectory(trajectory):
    global current
    if current is None:
        current = trajectory
    else:
        current = merge(current, trajectory)


def merge(a, b):
    """Marge a with b where b is a newer trajectory

    This assumes that both trajectories have
    """
    pass
