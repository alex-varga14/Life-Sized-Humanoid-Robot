from sys import argv
import roslaunch
import rospy
import sys

if __name__ == "__main__":
    rospy.init_node('motor_sensor', anonymous=True) #initializing node for motor sensors
    print("node init")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/glorycode/catkin_ws/src/gui_tutorials/launch/motor_sensor.launch"]) #roslaunching the motor_sensor launch file which will start up sensors.py
    launch.start() #starting the roslaunch
    rospy.loginfo("started")
    while not rospy.is_shutdown():
        rospy.sleep(3)
    # 3 seconds later
    launch.shutdown() #shutting down the roslaunch