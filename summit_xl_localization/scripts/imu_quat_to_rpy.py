#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf,math

def callback(data):
    r,p,y = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    rospy.loginfo("rpy: %.3lf, %.3lf, %.3lf(%.3lf deg)", r, p, y, math.degrees(y))
        
def listener():

    rospy.init_node('quat_to_rpy', anonymous=True)

    rospy.Subscriber("imu/data", Imu, callback)
                      
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
