#!/usr/bin/env python
"""
Connect to teensy. Protocol:
    To the teensy:
        <h/t/k><pwm value>\n
            where h/t/k is the joint and pwm value is +-1024
            the pwm value should observe both deadband (25%) and governer (40%)
            so values should range: -410, 256, 0, 256, 410
            + values will extend, - values will retract
    From the teensy:
        <h/t/k/c><string pot reading>\n
            where h/t/k/c is the sensor (c = compliant)
            and string pot reading is a 16 bit unsigned number

teensy must receive a command every 0.5 seconds or it will timeout.
timeout sets all pwm to 0

Connect to ros using:
    - Heartbeat client
    - publishing string pot values:
        topic = /<leg>/<joint>, int32
    - publishing pwm values [for logging]:
        topic = /<leg>/<joint> int32
    - listening to joystick values
        map joystick to pwms (different for each leg)
        buttons can determine if a single leg is 'active'
            x (+left) = extend knee
            y (+up) = extend thigh
            z (+cw) = extend/retract knee [per leg]
"""

import glob
import time

import serial

import rospy
import sensor_msgs.msg
import std_msgs.msg


leg_name = 'fr'
# index of button that should be pressed when issuing commands to just this leg
button_index = 4
pwm_min = 256
pwm_max = 410
joystick_min = 0.0
axis_map = {
    'hip': {
        'axis': 2,
        'flip': 1,
        'jmin': joystick_min,
        'pmin': pwm_min,
        'pmax': pwm_max,
    },
    'thigh': {
        'axis': 1,
        'flip': 1,
        'jmin': joystick_min,
        'pmin': pwm_min,
        'pmax': pwm_max,
    },
    'knee': {
        'axis': 0,
        'flip': 1,
        'jmin': joystick_min,
        'pmin': pwm_min,
        'pmax': pwm_max,
    },
}

monitor_delay = 0.01  # seconds to wait if no bytes to read
send_period = 2.5  # ms between pwm updates
joint_codes = {'H': 'hip', 'T': 'thigh', 'K': 'knee', 'C': 'calf'}
reverse_joint_codes = {joint_codes[k]: k for k in joint_codes}

joint_sensor_pubs = {'hip': None, 'thigh': None, 'knee': None, 'calf': None}
joint_pwm_pubs = {'hip': None, 'thigh': None, 'knee': None}
pwms = {'hip': 0, 'thigh': 0, 'knee': 0}
last_pwms_time = None
conn = None


def calculate_pwms(axes):
    global pwms
    for j in axis_map:
        m = axis_map[j]
        a = axes[m['axis']]
        # if joystick is in it's deadband, do nothing
        if abs(a) < m['jmin']:
            pwms[j] = 0.
            continue
        # compute pwm range
        pscale = (m['pmax'] - m['pmin']) / (1. - m['jmin'])
        if a > 0:
            pwms[j] = int((a - m['jmin']) * pscale + m['pmin'])
        else:
            pwms[j] = int((a + m['jmin']) * pscale - m['pmin'])


def publish_pwms():
    global pwms
    for j in pwms:
        p = joint_pwm_pubs[j]
        p.publish(pwms[j])


def write_pwms():
    global pwms
    global last_pwms_time
    for p in pwms:
        code = reverse_joint_codes[p]
        value = pwms[p]
        #conn.write('%s%i\n' % (code, value))
        print("%s%i" % (code, value))
    last_pwms_time = time.time()


def new_joy(data):
    # check for 'deadman', if not pressed, send 0 pwms
    global pwms
    if data.buttons[0] == 0:
        print("Deadman is open!")
        pwms = {'hip': 0, 'thigh': 0, 'knee': 0}
        return
    # check for 'me' button or no buttons
    if any(data.buttons[1:]) and data.buttons[button_index] == 0:
        print(
            "Other button[%s] is pressed, doing nothing"
            % (data.buttons[1:], ))
        pwms = {'hip': 0, 'thigh': 0, 'knee': 0}
        return
    # map data.axes to pwm
    calculate_pwms(data.axes)
    # publish pwm
    publish_pwms()
    # acquire teensy lock?
    # write new pwms to teensy
    print("writing pwms: %s" % (pwms, ))
    write_pwms()


def find_port(port):
    if port is not None:
        return port
    ports = glob.glob('/dev/ttyACM*')
    if len(ports) != 1:
        raise Exception("Failed to find port: %s" % (ports, ))
    return ports[0]


def connect_to_teensy(port=None):
    global conn
    #conn = serial.Serial(find_port(port), 115200)


def connect_to_ros():
    rospy.init_node(leg_name)
    rospy.Subscriber('/joy', sensor_msgs.msg.Joy, new_joy)
    for j in joint_sensor_pubs:
        joint_sensor_pubs[j] = rospy.Publisher(
            '/%s/sensors/%s' % (leg_name, j), std_msgs.msg.Int32,
            queue_size=16)
    for j in joint_pwm_pubs:
        joint_pwm_pubs[j] = rospy.Publisher(
            '/%s/pwms/%s' % (leg_name, j), std_msgs.msg.Int32,
            queue_size=16)


def monitor_teensy():
    # monitor teensy, publish new pot messages when received
    global last_pwms_time
    last_pwms_time = time.time()
    while True:
        if time.time() - last_pwms_time > send_period:
            # send pwms
            write_pwms()
        """
        if conn.inwaiting:
            msg = conn.readline()
            if len(msg) < 2 or msg[0] not in joint_codes:
                print("invalid message: %s" % (msg, ))
                continue
            joint = joint_codes[msg[0]]
            try:
                value = msg[1:]
            except ValueError:
                print("failed to parse message: %s" % (msg, ))
            joint_sensor_pubs[joint].publish(value)
        else:
            rospy.sleep(monitor_delay)
        """
        # TODO remove this when writing to teensy
        rospy.sleep(monitor_delay)


if __name__ == '__main__':
    connect_to_teensy()
    connect_to_ros()
    monitor_teensy()
