#!/usr/bin/env python3

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_examples read_write_node.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

def getch():
    return sys.stdin.read(1)

# Control table address
ADDR_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 564
ADDR_PRESENT_POSITION   = 580
ADDR_HW_ERROR		 = 518

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_11                      = 11                 # Dynamixel ID : 1
DXL_ID_13                      = 13                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable rSetPositionange. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

left_arm = [2,4,6,8,10,12,14]
right_arm = [1,3,5,7,9,11,13]
left_leg = [16,18,20,22,24,26]
right_leg =[15,17,19,21,23,25]
torso = [27,28,29]
left_hand = [32,34,36]
right_hand = [31,33,35]
head = [41,42]

def set_goal_pos_callback(data):
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)

def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position
    
def get_hw_error(req):
    dxl_hw_error, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_HW_ERROR)
    print("Present Position of ID %s = %s" % (req.id, dxl_hw_error))
    return dxl_hw_error

def read_write_py_node():
    rospy.init_node('read_write_py_node')
    rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)
    rospy.Service('get_position', GetPosition, get_present_pos)
    #rospy.Service('get_hw_error', GetHwError, get_hw_error)
    rospy.spin()

def main():
    # Open port
    try:
       print(rospy.get_param_names())
    except ROSException:
       print("could not get param name")
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # # Enable Dynamixel Torque
    # for i in head:
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")
    
    for i in left_arm:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            # print("Press any key to terminate...")
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            getch()
            # quit()
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            # print("Press any key to terminate...")
            print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
            getch()
            # quit()
        else:
            print("DYNAMIXEL has been successfully connected")

    # for i in right_arm:
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")

    # for i in right_leg:
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")

    # for i in left_leg:
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")

    # for i in torso:
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #         # print("Press any key to terminate...")
    #         print("WARNING MOTOR ", i, " NOT CONNECTED!!!")
    #         getch()
    #         # quit()
    #     else:
    #         print("DYNAMIXEL has been successfully connected")
   
    
    print("Ready to get & set Position.")

    read_write_py_node()

    

if __name__ == '__main__':
    main()











#do not uncomment out!!

 # dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_11, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     print("Press any key to terminate...")
    #     getch()
    #     quit()
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     print("Press any key to terminate...")
    #     getch()
    #     quit()
    # else:
    #     print("DYNAMIXEL has been successfully connected")
        

    # dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_13, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     # print("Press any key to terminate...")
    #     print("WARNING MOTOR ", DXL_ID_13, " NOT CONNECTED!!!")
    #     getch()
    #     # quit()
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     # print("Press any key to terminate...")
    #     print("WARNING MOTOR ", DXL_ID_13, " NOT CONNECTED!!!")
    #     getch()
    #     # quit()
    # else:
    #     print("DYNAMIXEL has been successfully connected")