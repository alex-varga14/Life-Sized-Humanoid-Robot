from kivy.app import App
from kivy.lang import Builder
import sys
import rospy
from kivy.core.window import Window
import time
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import * 
from threading import Thread
import subprocess
import roslaunch
from sensor import *
import re
import time

def LA_button():
    command1= "rqt_plot /linear_acceleration_X:linear_acceleration_Y:linear_acceleration_Z"
    p = subprocess.run(command1, shell=True) 

def AV_button():
    command1= "rqt_plot /angular_velocity_X:angular_velocity_Y:angular_velocity_Z"
    p = subprocess.run(command1, shell=True) 

def Rviz_button():
    command2= "rosrun rviz rviz -f vectornav"
    p = subprocess.run(command2, shell=True) 
     
    
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
    SPEED                  = 1
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

    for i in right_leg:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler4, i, ADDR_GOAL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler4, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
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

    for i in left_leg:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, i, ADDR_VEL_POSITION, SPEED)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
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
    
    for i in left_hand:
        dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler1, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler1, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")
    
    for i in right_hand:
        dxl_comm_result, dxl_error = packetHandler2.write2ByteTxRx(portHandler2, i, ADDR_VEL_POSITION_HANDS, SPEED_HANDS)
        dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler2, i, ADDR_TORQUE_ENABLE_HANDS, TORQUE_ENABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, i, ADDR_ACC, ACC)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            # getch()
        else:
            print("DYNAMIXEL has been successfully connected")


    App.get_running_app().stop
    Window.close()
    
    command = "rosnode kill --all" #once the gui has exited, then kill all rosnodes
    p = subprocess.run(command, shell=True) 
    roscore.terminate() #close roscore
   

class RobotSection(App):

    #will load up the seperate popup windows for each limb
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if limb == 'Head':
            self.screen=Builder.load_file('head_gui.kv') #launching the seperate .kv files based on the argument passed from the simple_gui.py file
        elif limb == 'Left Arm':
            self.screen=Builder.load_file('left_arm_gui.kv') #launching the seperate .kv files based on the argument passed from the simple_gui.py file
        elif limb == 'Right Arm':
            self.screen=Builder.load_file('right_arm_gui.kv') #launching the seperate .kv files based on the argument passed from the simple_gui.py file
        elif limb == 'Torso':
            self.screen=Builder.load_file('torso_gui.kv') #launching the seperate .kv files based on the argument passed from the simple_gui.py file
        elif limb == 'Left Leg':
            self.screen=Builder.load_file('left_leg_gui.kv')  #launching the seperate .kv files based on the argument passed from the simple_gui.py file           
        elif limb == 'Right Leg':
            self.screen=Builder.load_file('right_leg_gui.kv') #launching the seperate .kv files based on the argument passed from the simple_gui.py file
        elif limb == 'IMU':
            self.screen=Builder.load_file('imu_gui.kv')  #launching the seperate .kv files based on the argument passed from the simple_gui.py file               
    
    def build(self):
        return self.screen

    #thread function which greys out the publish button for 3 seconds when it is pressed
    #prevents the rest of the gui from sleeping when time.sleep is called
    def send_position_callback(self, id, position, velocity, buttonid):
        pub.publish(int(id), int(position),int(velocity))
        self.screen.ids[buttonid].disabled = True #disables the buttons
        time.sleep(3) #disbaled for 3 seconds
        self.screen.ids[buttonid].disabled = False

    #function which converts degrees to the appropriate position values for the motors based on their types
    def send_position(self, id, position, velocity, buttonid):
        print("Button pressed")
        id_i = int(id) #casting the id to an int 
        if ((id_i > 0 and id_i < 9) or (id_i==15) or (id_i == 16) or (id_i == 27)): #converts degrees to appropriate position value
            position = ((int(position)*(303750 *2))/360) #initial position *2/360
            # velocity = 750
           # position = ((int(position)*(501433*2))/360) #initial position *2/360
        elif ((id_i > 8 and id_i < 15) or (id_i==28) or (id_i == 29)):
            position = ((int(position)*(303454*2))/360) #initial position *2/360
            # velocity = 750
        elif((id_i > 30 and id_i < 37)):    #ids for hands
            position = ((int(position)*(1023))/300)
            # velocity = 100
        print("position = ", position, "velocity = ", velocity)#
        thread1 = Thread(target=self.send_position_callback, args=(id, position, velocity, buttonid)) #starts the thread which controls the button functionality
        thread1.setDaemon(True)
        thread1.start()
        # thread2 = Thread(target=self.send_position_callback, args=(id, position,buttonid))
        # thread2.start()
    
    def wave_left_callback(self, id, position, buttonid):
        pub.publish(int(id), int(position))
        self.screen.ids[buttonid].disabled = True #disables the buttons
        time.sleep(3) #disbaled for 3 seconds
        self.screen.ids[buttonid].disabled = False
   

    def test(self, id):
        print(str(id)+" was checked")
        m = "id: " +str(id)
        # thread1 = Thread(target=self.send_position_callback, args=(m,))
        # thread1.start()

        #sending the command line argument to start up the service through a subprocess
        process = subprocess.Popen(['rosservice', 'call', '/get_motor_sensors', id], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        i = 0
        out = [0]*3
        for line in process.stdout:
            out[i] = (re.sub('[^0-9]','', line.decode("ascii"))) #gets rid of any extra characters
            i+=1

        #displays the values of the current, voltage and temp to the gui screen
        self.screen.ids.current_id.text = str(out[0])    
        self.screen.ids.voltage_id.text = str(float(out[1])/10)
        self.screen.ids.temperature_id.text = str(out[2])

    def emergency(self):
        print("emergency_stop pressed")
        threatStart = Thread(target = stop_button, args=())
        threatStart.start()
        
#IMU buttons functionaities.
    def linear_acc(self, id):
        threadStart = Thread(target= LA_button, args=())
        threadStart.start()

    def angular_vel(self, id):
        threadStart = Thread(target= AV_button, args=())
        threadStart.start()
    
    def imu_rviz(self, id):
        threadStart = Thread(target= Rviz_button, args=())
        threadStart.start()     


if __name__ == "__main__":
    
    limb = str(sys.stdin.read())
    rospy.init_node('motors', anonymous=True)
    pub = rospy.Publisher('set_position', SetPosition, queue_size=10)
    print("robot section says limb = ", limb)

    RobotSection().run()
    