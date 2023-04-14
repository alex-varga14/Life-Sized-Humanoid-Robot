#!/usr/bin/env python3.8
from pickletools import uint8
from std_msgs.msg import Bool
from kivymd.app import MDApp
from kivy.app import App
from kivy.lang import Builder
from kivy.core.window import Window
import time
import rospy
import os
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from kivy.uix.popup import Popup
from kivy.uix.floatlayout import FloatLayout
import subprocess
import sys
from threading import *
from roscore import *
import rosgraph
from kivy.core.window import Window
from robot_section import *

#arrays for the motors ids of arms for demo purposes
left_arm = [2,4,6,8,10,12,14]
right_arm = [1,3,5,7,9,11,13]
#roscore = Roscore()


#initializing the motors using a seperate thread
#subprocess open up the python file which starts the launch file
def init_motors():
    print("initing mtors")
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/read_write_node_launch.py"])).start()

#initializing motor sensors 
#subprocess open up the python file which starts the launch file
def init_motor_sensors():
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/sensors_launch.py"])).start()

def init_force():
    command = "roslaunch rokubimini_serial rokubimini_serial.launch"
    p = subprocess.run(command, shell=True)

def init_velo():
    command = "roslaunch velodyne_pointcloud VLP16_points.launch"
    p = subprocess.run(command, shell=True)


#initializing the other sensors
#subprocess open up the python file which starts the launch file
def init_sensors():
    Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/all_sensors_launch.py"])).start()

#runs the command to open the velodyne visualization in rviz through a subprocess
def start_velo():
    print("collecting data")
    command = "rosrun rviz rviz -f velodyne"
    p = subprocess.run(command, shell=True)

#runs the command to open the camera visualization in rviz through a subprocess
def start_camera():
    print("collecting data")
    command = "roslaunch realsense2_camera demo_t265.launch"
    p = subprocess.run(command, shell=True)

#runs the command to record the force torque data to a rosbag file for later processing
def start_force():
    print("collecting data")
    command = "rosbag record -O forceTorque /bus0/ft_sensor0/ft_sensor_readings/wrench"
    p = subprocess.run(command, shell=True)

def stop_button():
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
    PROTOCOL_VERSION_HANDS      = 1.0
    DEVICENAME1  = '/dev/ttyUSB0'    # Check which port is being used on your controller using the command "ls -l /dev/ttyUSB*"
    DEVICENAME2  = '/dev/ttyUSB1'
    DEVICENAME3  = '/dev/ttyUSB2'
    DEVICENAME4  = '/dev/ttyUSB3'

    portHandler1 = PortHandler(DEVICENAME1)             # left arm, head and torso     
    portHandler2 = PortHandler(DEVICENAME2)          	# right arm
    portHandler3 = PortHandler(DEVICENAME3)             # left leg
    portHandler4 = PortHandler(DEVICENAME4)         	# right leg
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    packetHandler2 = PacketHandler(PROTOCOL_VERSION_HANDS)

    try:
        portHandler1.openPort()
        portHandler2.openPort()
        portHandler3.openPort()
        portHandler4.openPort() #comment the porthandler corresponding to the port that is not being used
    except:
        print("failed to connect port in stop_button")
        getch()
        quit()

    try:
        portHandler1.setBaudRate(BAUDRATE)
        portHandler2.setBaudRate(BAUDRATE)
        portHandler3.setBaudRate(BAUDRATE)
        portHandler4.setBaudRate(BAUDRATE)  #comment the porthandler corresponding to the port that is not being used

    except:
        print("Failed to change the stop_button baudrate")
        getch()
        quit()

    #arrays which contain the motor ids for their corresponding sections
    head = [41,42]
    left_arm = [2,4,6,8,10,12,14]
    right_arm = [1,3,5,7,9,11,13]
    left_leg = [16,18,20,22,24,26]
    right_leg =[15,17,19,21,23,25]
    torso = [27,28,29]
    left_hand = [32,34,36]
    right_hand = [31,33,35]

    TORQUE_ENABLE = 0
    ADDR_TORQUE_ENABLE = 512
    ADDR_VEL_POSITION      = 560
    SPEED                  = 2
    ADDR_VEL_POSITION_HANDS      = 32
    SPEED_HANDS = 1
    ADDR_ACC               = 556
    ACC                    = 0
    
    for i in head:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
    for i in left_arm:
        print("left arm")
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_VEL_POSITION, SPEED)
        print("Set Velocity of ID %s = %s" % (i, SPEED))
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in right_arm:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in right_leg:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler4, i, ADDR_GOAL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler4, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler4, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in left_leg:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")

    for i in torso:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        print("dxl_comm_result, dxl_error", dxl_comm_result, dxl_error, COMM_SUCCESS)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            getch()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
    # for i in left_hand:
    #     dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler1, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
    #     dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
    #      
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler2.getRxPacketError(dxl_error))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")
    
    # for i in right_hand:
    #     dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler2, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
    #     dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler2, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
    #      
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler2.getRxPacketError(dxl_error))
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         # getch()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")


    App.get_running_app().stop
    Window.close()
    
    command = "rosnode kill --all" #once the gui has exited, then kill all rosnodes
    p = subprocess.run(command, shell=True) 
    roscore.terminate() #close roscore

def launch_button():
    cmd1="roslaunch vectornav vectornav.launch"
    p = subprocess.run(cmd1, shell=True)
    
class P(FloatLayout):
    pass

#launches the popup window for the motor sliders
def show_popup():
    show = P()
    time.sleep(3)
    popupWindow.open()

class TutorialApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        self.screen=Builder.load_file('ros_gui1.kv') #load the .kv file for the main window
    
    def build(self):
        return self.screen
    
    def reset_all(self):	
        pass
    
    #in a seperate thread to prevent main window from freezing visaluze the velodyne data
    def velo_bag(self):
        threatStart = Thread(target = init_velo, args=())
        threatStart.start()
        thread2 = Thread(target=start_velo, args=()) 
        thread2.setDaemon(True)
        thread2.start()
    
    #in a seperate thread to prevent main window from freezing visualize the camera data
    def camera_bag(self):
        thread4 = Thread(target=start_camera, args=()) 
        thread4.setDaemon(True)
        thread4.start()

    #in a seperate thread to prevent main window from freezing bag the force torque data
    def force_bag(self):
        threadStart = Thread(target = init_force, args=())
        threadStart.start()
        thread5 = Thread(target=start_force, args=()) 
        thread5.setDaemon(True)
        thread5.start()
        
    # #thread function which greys out the publish button for 3 seconds when it is pressed
    # #prevents the rest of the gui from sleeping when time.sleep is called
    # def send_position_callback(self, id, position, velocity, buttonid):
    # 	pub.publish(int(id), int(position), int(velocity))
    # 	self.screen.ids[buttonid].disabled = True #disables the buttons
    # 	time.sleep(10) #disbaled for 3 seconds
    # 	self.screen.ids[buttonid].disabled = False

    # #function which converts degrees to the appropriate position values for the motors based on their types
    # def send_position(self, id, position, buttonid):
    # 	print("Button pressed")
    # 	id_i = int(id) #casting the id to an int 
    # 	if ((id_i > 0 and id_i < 9) or (id_i==15) or (id_i == 16) or (id_i == 27)): #converts degrees to appropriate position value
    # 		position = ((int(position)*(501433*2))/360) #initial position *2/360
    # 	elif ((id_i > 8 and id_i < 15) or (id_i==28) or (id_i == 29)):
    # 		position = ((int(position)*(303454*2))/360) #initial position *2/360
    # 	print("position = ", position)
    # 	thread1 = Thread(target=self.send_position_callback, args=(id, position,buttonid)) #starts the thread which controls the button functionality
    # 	thread1.setDaemon(True)
    # 	thread1.start()
    # 	# thread2 = Thread(target=self.send_position_callback, args=(id, position,buttonid))
    # 	# thread2.start()

    #thread function which contains the sequence to move the right arm in a wave
    def wave_right_arm_callback(self, buttonid):
        print("waving right arm")
        pub.publish(int(right_arm[1]), int((20*(501433*2))/360),int(750)) #moving the motors to hardcoded value, converting the angle from sliders to a positional value for the motors
        rospy.sleep(1)
        pub.publish(int(right_arm[3]), int((-77*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((-1*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[5]), int((-71*(303454*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(right_arm[4]), int((29*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((-29*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((29*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[1]), int((20*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[3]), int((-16*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[4]), int((0*(303454*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(right_arm[5]), int((0*(303454*2))/360),int(750))
        self.screen.ids[buttonid].disabled = True  #disables the button to keep users from repeatedly pushing while the movement sequence is taking place
        time.sleep(10) #keep disabled for 10 seconds
        self.screen.ids[buttonid].disabled = False
    
    #demo right arm wave through a thread to keep main window from freezing while demo takes place
    def wave_right_arm(self, buttonid):
        thread1 = Thread(target=self.wave_right_arm_callback, args=(buttonid,)) #starts the thread which controls the button functionality
        thread1.start()

    #thread function which contains the sequence to move the left arm in a wave
    def wave_left_arm_callback(self, buttonid):
        print("waving left arm")
        pub.publish(int(left_arm[1]), int((-90*(501433*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(left_arm[2]), int((-90*(501433*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(left_arm[3]), int((67*(501433*2))/360),int(750))
        rospy.sleep(2)
        pub.publish(int(left_arm[3]), int((51*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((67*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((51*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((67*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((51*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[1]), int((-20*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[2]), int((0*(501433*2))/360),int(750))
        rospy.sleep(1)
        pub.publish(int(left_arm[3]), int((0*(501433*2))/360),int(750))
        self.screen.ids[buttonid].disabled = True  #disables the button to keep users from repeatedly pushing while the movement sequence is taking place
        time.sleep(10) #keep disabled for 10 seconds
        self.screen.ids[buttonid].disabled = False

    # # demo left arm wave through a thread to keep main window from freezing while demo takes place
    def wave_left_arm(self, buttonid):
        thread1 = Thread(target=self.wave_left_arm_callback, args=(buttonid,)) #starts the thread which controls the button functionality
        thread1.start()

    #launches the seperate the popup windows for the motor sliders with different threads
    @staticmethod
    def motor_section_show(id):
        print("New window open")
        if str(id) == 'Head': #if the button pressed on main window is 'Head' pass robot_section.py 'Head' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Head")).start() #starting up the code for each different section in a seperate thread
        elif str(id) == 'Left Arm': #if the button pressed on main window is 'Left Arm' pass robot_section.py 'Left Arm' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Left Arm")).start()
        elif str(id) == 'Right Arm': #if button pressed on main window is 'Right Arm' pass robot_section.py 'Right Arm' as argumeny
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Right Arm")).start()
        elif str(id) == 'Torso': #if button pressed on main window is 'Torso' pass robot_section.py 'Torso' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Torso")).start()
        elif str(id) == 'Left Leg': #if button pressed on main window is 'Left Leg' launch the left leg popup pass robot_section.py 'Left Leg' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"Left Leg")).start()
        elif str(id) == 'Right Leg': #if the button pressed on main window is 'Right Leg' pass robot_section.py 'Right Leg' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, "/home/jia/catkin_ws/src/gui_tutorials/src/robot_section.py"], input=b"Right Leg")).start()
        elif str(id) == 'IMU': #if the button pressed on main window is 'Left Arm' pass robot_section.py 'Left Arm' as argument
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/robot_section.py"], input=b"IMU")).start()
            threatStart = Thread(target = launch_button, args=())
            threatStart.start()
            Thread(target=lambda *largs: subprocess.run([sys.executable, os.getcwd() + "/src/gui_tutorials/src/imu_subscriber.py"])).start()
            


    def emergency(self):
        print("emergency_stop is pressed")
        threatStart = Thread(target = stop_button, args=())
        threatStart.start() 


if __name__ == '__main__':	

    
    # if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
    #     print('ROS MASTER is Online')
    #     roscore.terminate()
    # else:
    #     print('ROS MASTER is Offline') #if ros master is offline then start a new instance of roscore
    # roscore.run()
    # time.sleep(2)
    
    rospy.init_node('jia_gui', anonymous=True)
    pub = rospy.Publisher('set_position', SetPosition, queue_size=10)

    init_motors() #initializing the motors
    init_motor_sensors() #initializing the motor sensors
    # init_sensors() #initializing the sensors

    TutorialApp().run() #running the gui
    command = "rosnode kill --all" #once the gui has exited, then kill all rosnodes
    p = subprocess.run(command, shell=True) 
    roscore.terminate() #close roscore
