#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
global imu_data

def imucallback(msg):
	imu_data = msg.linear_acceleration
	rospy.loginfo("I heard %s", imu_data)   
	
	
def listener():
	rospy.init_node('listener', anonymous=True) 
	rospy.Subscriber("/vectornav/IMU", Imu, imucallback)
	rospy.spin()

if __name__ == '__main__':
	listener()

