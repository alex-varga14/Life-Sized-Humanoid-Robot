from kivy.app import App
from kivy.lang import Builder
import sys
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import * 
from threading import Thread
import subprocess
import roslaunch
from sensor import *
import re
import time

def send_position_callback(id, position):
        pub.publish(int(id), int(position))
        # self.screen.ids[buttonid].disabled = True #disables the buttons
        # time.sleep(3) #disbaled for 3 seconds
        # self.screen.ids[buttonid].disabled = False

if __name__ == "__main__":
    
    arm = str(sys.stdin.read())
    print(arm)
    pub = rospy.Publisher('set_position', SetPosition, queue_size=10)
    rospy.init_node('simple_gui', anonymous=True)

    thread1 = Thread(target=send_position_callback, args=(4, (-90*(501433*2))/360),) #starts the thread which controls the button functionality
    thread1.setDaemon(True)
    thread1.start()