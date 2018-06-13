#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TwistStamped


if __name__ == '__main__':
    rospy.init_node('ur_cmd_publisher', anonymous=True)

    pub = rospy.Publisher('jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=1)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

        try:
            (trans_ee, rot_ee) = listener.lookupTransform('/base_link','/ee_link', rospy.Time(0))
            (trans_marker, rot_marker) = listener.lookupTransform('/base_link', 'marker_10', rospy.Time(0))
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) :
            continue
        trans_diff = [0] * 3

        for index in range(len(trans_marker)):
            trans_diff[index] = trans_marker[index] - trans_ee[index]

        ts = TwistStamped()

        ts.header.stamp = rospy.Time.now()

        ts.header.frame_id = rospy.get_param("joy_to_twist/cmd_frame")

        if ( (trans_diff[0] * trans_diff[0] + trans_diff[1] * trans_diff[1] + trans_diff[2] * trans_diff[2]) > 0.01):
            if  trans_diff[0] > 0.3:
                ts.twist.linear.x = -0.3
            elif trans_diff[0] < -0.3:
                ts.twist.linear.x = 0.3
            else:
                ts.twist.linear.x = -trans_diff[0]
        
            if  trans_diff[1] > 0.3:
                ts.twist.linear.y = -0.3
            elif trans_diff[1] < -0.3:
                ts.twist.linear.y = 0.3
            else:
                ts.twist.linear.y = -trans_diff[1]
            
            if  trans_diff[2] > 0.3:
                ts.twist.linear.z = 0.3
            elif trans_diff[2] < -0.3:
                ts.twist.linear.z = -0.3
            else:
                ts.twist.linear.z = trans_diff[2]
        
        else:
            ts.twist.linear.x = 0.0
            ts.twist.linear.y = 0.0
            ts.twist.linear.z = 0.0
        
        ts.twist.angular.x = 0.0
        ts.twist.angular.y = 0.0
        ts.twist.angular.z = 0.0

        pub.publish(ts)
