#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String



if __name__ == '__main__':
    rospy.init_node('ur_cmd_publisher', anonymous=True)

    pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    stop_cmd = String()
    stop_cmd.data = 'stopl(0.2)'

    while not rospy.is_shutdown():

        try:
            (trans_ee, rot_ee) = listener.lookupTransform('/base','/tool0', rospy.Time(0))
            (trans_marker, rot_marker) = listener.lookupTransform('/base', 'marker_10', rospy.Time(0))
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) :
            continue
        trans_diff = [0] * 3

        for index in range(len(trans_marker)):
            trans_diff[index] = trans_marker[index] - trans_ee[index]
        
        cmd = [0] * 3
        scale = 0.1
        if (trans_diff[0] * trans_diff[0] + trans_diff[1] * trans_diff[1] + trans_diff[2] * trans_diff[2]) < 0.04 :
            pub.publish(stop_cmd)
            print "stop_cmd published"
        else:
            for index in range(len(trans_diff)):
                if trans_diff[index] > 0.2 :
                    cmd[index] = 0.2 * scale
                elif trans_diff[index] < -0.2 :
                    cmd[index] = -0.2 * scale
                else:
                    cmd[index] = trans_diff[index] * scale


        cmd = 'movel([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], 0.2, 0.1)\n' % (cmd[0], cmd[1] , cmd[2], 0., 0., 0.)
        #print cmd
        cmd_script = String()
        cmd_script.data = cmd
        
        #pub.publish(cmd_script)

        print rot_ee

        rate.sleep()
        


