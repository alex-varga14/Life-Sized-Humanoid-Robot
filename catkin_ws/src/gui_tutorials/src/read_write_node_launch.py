from sys import argv
import roslaunch
import rospy
import sys


def launch_all():
    print("in the launch python file")
    rospy.init_node('read_write_node', anonymous=True) #initializing the read_write_node for the motors 
    print("node init")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/glorycode/catkin_ws/src/gui_tutorials/launch/read_write_node.launch"]) #roslaunching the read_write_node launch file
    launch.start() #starting roslaunch
    rospy.loginfo("started")
    while not rospy.is_shutdown():
        rospy.sleep(3)
    # 3 seconds later
    launch.shutdown()


if __name__ == "__main__":
    print("in main")
    launch_all()


