#!/usr/bin/env python
"""
Trace a path with a leg
  start position (x, y, z) or blank to read out
  end position (x, y, z)
  time to move (seconds) rate to update (hz)


(read out start position)
compute vector from start to end
break up vector wrt time and rate
start movement...
"""

import argparse

import rospy

import stompy.ros


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-s', '--start', default="", type=str)
    p.add_argument('-e', '--end', default="1.8,0.0,0.5", type=str)
    p.add_argument('-t', '--time', default=5., type=float)
    p.add_argument('-r', '--rate', default=10., type=float)
    args = p.parse_args()

    stompy.ros.init.init('stompy_trace')
    pts = [
        (1.8, 0, 0.5),
        (1.2, 1., 0.8),
        (1.4, -1., 0.2),
    ]
    pi = 0
    sleep_time = 1. / args.rate
    print("Waiting for connection...")
    rospy.sleep(1.)
    state = 0
    while not rospy.is_shutdown():
        if state == 0:
            # (read out start position)
            if stompy.sensors.joints.joints is not None:
                msg = stompy.ros.trajectories.line(
                    'fl',
                    stompy.sensors.joints.legs['fl']['foot'],
                    pts[pi])
                stompy.ros.legs.publishers['fl'].send_goal(msg)
                state = 1
        else:  # goal should have been sent
            p = stompy.ros.legs.publishers['fl']
            if p.get_state() == 1:
                pass
            elif p.get_state() == 3:
                print("Done moving")
                pi += 1
                state = 0
                if pi >= len(pts):
                    pi = 0
            else:
                raise Exception("%s" % p.get_state())
        rospy.sleep(sleep_time)

if __name__ == '__main__':
    main()
