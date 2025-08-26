#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class StatetManager:
    def __init__(self):
        rospy.init_node('state_manager', anonymous=True)
        self.mode_pub = rospy.Publisher('mode', String, queue_size=1)
        
        # Always publish "HIGHWAY MODE"
        mode_msg = String()
        mode_msg.data = "HIGHWAY MODE"
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.mode_pub.publish(mode_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        StatetManager()
    except rospy.ROSInterruptException:
        pass