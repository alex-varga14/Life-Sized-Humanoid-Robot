from sys import argv
import roslaunch
import rospy
import sys

#this file will start the launch file for all sensors which is required to create their topics and read data from them through ros

if __name__ == "__main__":
    rospy.init_node('all_sensor', anonymous=True) #initializing a ros node
    print("node init")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/glorycode/catkin_ws/src/sensors/launch/vectornav.launch"]) #roslaunching all_seonsor launch file 
    # launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/humanoidteam/catkin_ws/src/sensors/launch/rokubimini_serial.launch"])
    launch.start() #starting the roslaunch
    rospy.loginfo("started")
    while not rospy.is_shutdown(): #
        rospy.sleep(3)
    # 3 seconds later
    launch.shutdown() #shutting down the launch file