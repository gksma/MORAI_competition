#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from global_path_a_star import a_star_path_pub
from nav_msgs.msg import Path
from std_msgs.msg import String

class CombinedPathPub:
    def __init__(self):
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        rospy.Subscriber("/mode", String, self.mode_callback)
        
        self.a_star_planner = a_star_path_pub()
        self.mode = None
        
    def publish_global_path(self):
        # Calculate A* path once
        a_star_path = self.a_star_planner.calc_a_star_path_node(self.a_star_planner.node_list)
        
        # Set frame ID
        a_star_path.header.frame_id = '/map'
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # If the mode is "HIGHWAY MODE", publish the A* path
            if self.mode == "HIGHWAY MODE":
                self.global_path_pub.publish(a_star_path)
            rate.sleep()
    
    def mode_callback(self, msg):
        self.mode = msg.data

if __name__ == '__main__':
    rospy.init_node('combined_path_pub', anonymous=True)
    path_publisher = CombinedPathPub()
    path_publisher.publish_global_path()