#!/usr/bin/env python

import sys

import rosbag
import rospy


bag_fn = 'fr_2016-09-15-19-49-27.bag'
if len(sys.argv) > 1:
    bag_fn = sys.argv[1]


topic = '/stompy/fr/follow_joint_trajectory/goal'
if len(sys.argv) > 2:
    topic = sys.argv[2]

bag = rosbag.Bag(bag_fn)
tti = bag.get_type_and_topic_info()
if topic not in tti.topics:
    raise Exception("topic[%s] not found in bag files[%s]" % (topic, bag_fn))
ti = tti.topics[topic]
msg_type_str = ti.msg_type
module_name, msg_name = msg_type_str.split('/')
module = __import__(module_name + '.msg')
msg_type = getattr(module.msg, msg_name)

rospy.init_node('bag_publisher', anonymous=True)
pub = rospy.Publisher(topic, msg_type, queue_size=16)

bag_st = rospy.Time(bag.get_start_time())
ros_st = rospy.Time.now()
print("Start time: %s" % ros_st)


def restamp(msg, attribute, bag_start, ros_start, check_zero=True):
    sa = msg
    if '.' in attribute:
        for a in attribute.split('.')[:-1]:
            if hasattr(sa, a):
                sa = getattr(sa, a)
            else:
                return msg
        attribute = attribute.split('.')[-1]
    sv = getattr(sa, attribute)
    if check_zero and sv.is_zero():
        return msg
    nv = ros_start + (sv - bag_start)
    print("Restamping %s to %s" % (attribute, nv))
    setattr(sa, attribute, nv)
    return msg


for msg in bag.read_messages(topics=topic):
    # wait for message start time
    msg_dt = msg.timestamp - bag_st
    message = msg.message
    message = restamp(message, 'header.stamp', bag_st, ros_st)
    message = restamp(message, 'goal_id.stamp', bag_st, ros_st)
    message = restamp(message, 'goal.trajectory.header.stamp', bag_st, ros_st)
    #message = clear(message, 'goal_id.id')
#    if (
#            hasattr(message, 'header') and hasattr(message.header, 'stamp')
#            and (not message.header.stamp.is_zero())):
#        # retimestamp message
#        h_st = message.header.stamp - bag_st
#        message.header.stamp = ros_st + h_st
#        print(
#            "re-timestamped the header to %s[%s]" % (
#                message.header.stamp, message.header.stamp - ros_st))
    ros_dt = rospy.Time.now() - ros_st
    sleep_dt = (msg_dt - ros_dt).to_sec() - 0.01
    if sleep_dt > 0.:
        print("not yet time to send message, sleeping: %s" % sleep_dt)
        rospy.sleep(sleep_dt)
    print("publishing message")
    pub.publish(message)
