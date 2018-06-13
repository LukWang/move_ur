#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import numpy
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class topic_to_action:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        self.client.wait_for_server()
        print "Connected to server"

    def call_back(self, traj):
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory = traj
        traj_goal.trajectory.joint_names = JOINT_NAMES
        
        self.client.send_goal(traj_goal)
        try:
            self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise


    def republish(self):
        rospy.Subscriber("/ur5_traj_repub/follow_joint_trajectory", JointTrajectory, self.call_back)
        rospy.spin()


if __name__ == '__main__':

    rospy.init_node("topic_to_action", anonymous=True, disable_signals=True)
    actioner = topic_to_action()
    actioner.republish()
