#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
#global imu_data

def imucallback(msg):
	global data1, data2, data3, data4, data5, data6
	linear_acceleration_X = msg.linear_acceleration.x
	linear_acceleration_Y = msg.linear_acceleration.y
	linear_acceleration_Z = msg.linear_acceleration.z
	angular_velocity_X = msg.angular_velocity.x
	angular_velocity_Y = msg.angular_velocity.y
	angular_velocity_Z = msg.angular_velocity.z

	data1 = linear_acceleration_X* 57.2958
	data2 = linear_acceleration_Y* 57.2958
	data3 = linear_acceleration_Z* 57.2958
	data4 = angular_velocity_X* 57.2958
	data5 = angular_velocity_Y* 57.2958
	data6 = angular_velocity_Z* 57.2958
	rospy.loginfo("I heard %s", msg) 
	#ospy.loginfo("I heard %s", data2) 
	#rospy.loginfo("I heard %s", data5)   
	
def talker():
	#rospy.init_node('talker', anonymous=True)
	global data1, data2, data3, data4, data5, data6
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub1.publish(data1)
		pub2.publish(data2)
		pub3.publish(data3)
		pub4.publish(data4)
		pub5.publish(data5)
		pub6.publish(data6)
		rate.sleep()

def listener():
	rospy.init_node('listener', anonymous=True) 
	rospy.Subscriber("/vectornav/IMU", Imu, imucallback)
	talker()
	rospy.spin()

if __name__ == '__main__':
	data1 = 0.0
	data2 = 0.0
	data3 = 0.0
	data4 = 0.0
	data5 = 0.0
	data6 = 0.0
	pub1 = rospy.Publisher("linear_acceleration_X", Float64, queue_size=10)
	pub2 = rospy.Publisher("linear_acceleration_Y", Float64, queue_size=10)
	pub3 = rospy.Publisher("linear_acceleration_Z", Float64, queue_size=10)
	pub4 = rospy.Publisher("angular_velocity_X", Float64, queue_size=10)
	pub5 = rospy.Publisher("angular_velocity_Y", Float64, queue_size=10)
	pub6 = rospy.Publisher("angular_velocity_Z", Float64, queue_size=10)
	listener()
