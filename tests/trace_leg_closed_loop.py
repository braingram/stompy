#!/usr/bin/env python
"""
Trace a path with a leg
  start position (x, y, z) or blank to read out
  end position (x, y, z)
  time to move (seconds)
  rate to update (hz)


(read out start position)
compute vector from start to end
break up vector wrt time and rate
start movement...
"""

import argparse
import sys
import time

#import rospy
#import sensor_msgs.msg
#import std_msgs.msg

import numpy
import serial

import stompy.kinematics.leg as leg
#from stompy.planners.trajectory import PathTracer
#import stompy.sensors.joints as joints

global fake_angles
fake_angles = (0, 84, 13)

bfn = int(time.time())
log_file = open("clm_%i.log" % bfn, "w")


def read_angles(conn):
    if conn is None:
        print("-> %s" % (fake_angles, ))
        a = fake_angles
    else:
        # ####\t###\t###\n  # degrees
        if not conn.inWaiting():
            return None
        while conn.inWaiting():
            l = conn.readline()
            if len(l.strip()) == 0:
                return None
            log_file.write(str(time.time()) + '\t0\t' + l.strip() + '\n')
            print("-> %s" % (l, ))
            a = map(float, l.strip().split('\t'))
    hip, thigh, knee = a
    thigh = numpy.radians(84. - thigh)
    knee = numpy.radians(13. - knee)
    hip = numpy.radians(hip)
    return hip, thigh, knee


def write_angles(conn, hip, thigh, knee):
    # invert thigh, +84 degrees
    thigh = 84. - numpy.degrees(thigh)
    knee = 13. - numpy.degrees(knee)
    hip = numpy.degrees(hip)
    if conn is None:
        global fake_angles
        fake_angles = (hip, thigh, knee)
        print("<- %s" % (fake_angles, ))
    else:
        # ####\t###\t###\n
        l = "%.4f\t%.4f\t%.4f\n" % (hip, thigh, knee)
        print("<- %s" % (l.strip(),))
        log_file.write(str(time.time()) + '\t1\t' + l.strip() + '\n')
        conn.write(l)


def verify(msg):
    print(msg)
    r = raw_input("do these look ok?")
    if not 'y' in r.lower():
        sys.exit(1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('-s', '--start', default="", type=str)
    #p.add_argument('-e', '--end', default="1.8,0.0,0.5", type=str)
    p.add_argument('-e', '--end', default="0.0,0.0,0.1", type=str)
    p.add_argument('-t', '--time', default=5., type=float)
    p.add_argument('-p', '--port', default='/dev/ttyACM0')
    p.add_argument('-b', '--baud', default=9600)
    #p.add_argument('-d', '--delta', default=numpy.radians(1))
    #p.add_argument('-c', '--close', default=numpy.radians(0.5))
    p.add_argument('-d', '--delta', default=0.03)
    p.add_argument('-c', '--close', default=0.02)
    p.add_argument('-f', '--fake', default=False, action='store_true')
    args = p.parse_args()

    #conn = serial.Serial(args.port, args.baud)
    if args.fake:
        conn = None
    else:
        conn = serial.Serial(args.port, args.baud)

    # read in joint angles
    # eat first line
    if conn is not None:
        conn.readline()
    a = None
    while a is None:
        a = read_angles(conn)
    hip, thigh, knee = a
    #na = (hip, thigh, knee + numpy.radians(4))
    #verify("new: %s" % (na, ))
    #write_angles(conn, *na)
    #sys.exit(0)
    # set starting point
    start_angles = numpy.array((hip, thigh, knee))
    start_point = numpy.array(leg.forward(*start_angles))
    verify("Starting angles: %s" % (numpy.degrees(start_angles),))
    verify("Starting angles: %s" % (start_angles,))
    verify("Starting point: %s" % (start_point,))
    end_point = start_point + map(float, args.end.strip('\\').split(','))
    #end_point = start_point + (0.1, 0., 0.)
    verify("End point: %s" % (end_point,))
    # set ending point
    end_angles = numpy.array(leg.inverse(*end_point))
    # set max delta angle
    delta_angles = end_angles - start_angles
    verify("Delta angles: %s" % (numpy.degrees(delta_angles),))
    # compute number of moves
    n_moves = int(
        numpy.floor(numpy.max(numpy.abs(delta_angles) / args.delta)))
    verify("N moves: %s" % (n_moves,))
    angle_steps = delta_angles / n_moves
    verify("Angle steps: %s" % (angle_steps,))
    # while loop
    current_angles = start_angles
    current_target = start_angles
    verify("Start?")
    # send target
    write_angles(conn, *current_target)
    i = 0
    while i < n_moves:
        # try to read angles
        a = read_angles(conn)
        if a is not None:
            # parse new angles
            current_angles = numpy.array(a)
        error = numpy.abs(current_angles - current_target)
        if numpy.max(error) < args.close:
            # target hit, move to next
            i += 1
            if i == n_moves:
                break
            current_target = start_angles + i * angle_steps
            # send target
            write_angles(conn, *current_target)
        else:
            # wait...
            time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    log_file.close()
