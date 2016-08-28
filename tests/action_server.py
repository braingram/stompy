#!/usr/bin/env python
"""
This server should implement something like this:
http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement

It needs to:
    - if an empty trajectory arrives, the goal is canceled
    - if a trajectory with a blank timestamp arrives, start it now
    - if a new trajectory (b) arrives during a trajectory (a), merge it:
        - if b start time is now, drop all later points from a
        - if start time is in future, drop all points after b start time
        - if b start time is in past:
          drop all later points for a, and earlier points for b

Also, it will need to buffer points sent to the teensy allowing:
    - sufficient data for the teensy for future waypoints
    - preemption of already buffered (sent) points
    - clearing/cancelation of trajectories

Finally, it will need to produce at a minimum a result and check for
various tolerances during movement. Ideally, it will also provide feedback
during movements (although the gazebo controller does not).
"""

import actionlib
import control_msgs.msg
import rospy
import trajectory_msgs.msg


class JointTrajectoryActionServer(object):
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback
    _result = control_msgs.msg.FollowJointTrajectoryResult

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, control_msgs.msg.FollowJointTrajectoryAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # execute action
        success = True

        # TODO prep feedback
        # TODO check names = hip, thigh, knee
        names = goal.trajectory.joint_names
        print("Joint names: %s" % names)
        # wait for start
        start_time = goal.trajectory.header.stamp
        if rospy.Time.now() < start_time:
            print("Waiting for start time: %s" % start_time)
            rospy.sleep(start_time - rospy.Time.now())
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                print("preempted...")
                return

        # send points to teensy, monitor movement
        for pt in goal.trajectory.points:
            # check for preemption (new trajectory, etc...)
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                print("preempted...")
                break
            print("Moving %s to %s" % (names, pt.positions))
            st = pt.time_from_start - (rospy.Time.now() - start_time)
            rospy.sleep(st)
            # TODO append feedback
            # TODO send feedback
            pass

        print("Finished")
        # TODO publish result
        if success:
            r = self._result()
            self._as.set_succeeded(r)


if __name__ == '__main__':
    rospy.init_node('action_server')
    print("starting action server: %s" % rospy.get_name())
    JointTrajectoryActionServer(rospy.get_name())
    print("spinning...")
    rospy.spin()
