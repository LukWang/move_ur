#!/usr/bin/env python

import rospy
from std_msgs.msg import String



if __name__ == '__main__':
    rospy.init_node('ur_cmd_publisher', anonymous=True)

    pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

    rate = rospy.Rate(10)
    cmd = String()
    #cmd.data = 'movel(p[-0.3, -0.2, 0.3, 2.36, -0.4, -1.35], 0.1, 0.2, 0)'
    cmd.data = 'speedl([0.0, 0.0, 0.0, 0.0, 0.1, 0.0], 0.1, 0.1)'

    
    while not rospy.is_shutdown():
        pub.publish(cmd)
        print cmd
        rate.sleep()



