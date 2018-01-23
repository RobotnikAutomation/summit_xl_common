#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf
import math

def callback(data):
    r,p,y = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    rospy.loginfo("x,y,z: %.3lf, %.3lf, %.3lf - rpy: %.3lf, %.3lf, %.3lf(%.3lf deg)", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, r, p, y, math.degrees(y))
        
def listener():

    rospy.init_node('quat_to_rpy', anonymous=True)

    rospy.Subscriber("robotnik_base_control/odom", Odometry, callback)
                      
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

