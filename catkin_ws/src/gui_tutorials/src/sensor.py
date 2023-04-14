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
from dynamixel_sdk_examples.srv import GetPosition, GetPositionResponse
from dynamixel_sdk_examples.srv import GetMotorSensors, GetMotorSensorsResponse
from dynamixel_sdk_examples.msg import *



# Control table address
ADDR_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 564
ADDR_PRESENT_POSITION   = 580
ADDR_HW_ERROR		    = 518
ADDR_PRESENT_VOLTAGE    = 592
ADDR_PRESENT_TEMP       = 594
ADDR_PRESENT_CURRENT    = 574

# Control table for hands
ADDR_TORQUE_ENABLE_HANDS      = 24              # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION_HANDS      = 30
ADDR_PRESENT_POSITION_HANDS   = 36
ADDR_PRESENT_VOLTAGE_HANDS    = 42
ADDR_PRESENT_TEMP_HANDS       = 43
ADDR_PRESENT_CURRENT_HANDS    = 574

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
PROTOCOL_VERSION_HANDS      = 1.0

# Default setting
DXL_ID_11                      = 11                 # Dynamixel ID : 1
DXL_ID_13                      = 13                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME1                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
DEVICENAME2                 = '/dev/ttyUSB1'
DEVICENAME3                 = '/dev/ttyUSB2'
DEVICENAME4                 = '/dev/ttyUSB3'
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable rSetPositionange. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler1 = PortHandler(DEVICENAME1)
portHandler2 = PortHandler(DEVICENAME2)
portHandler3 = PortHandler(DEVICENAME3)
portHandler4 = PortHandler(DEVICENAME4)
packetHandler = PacketHandler(PROTOCOL_VERSION)
packetHandler2 = PacketHandler(PROTOCOL_VERSION_HANDS)
head = [41,42]
left_arm = [2,4,6,8,10,12,14]
right_arm = [1,3,5,7,9,11,13]
left_leg = [16,18,20,22,24,26]
right_leg =[15,17,19,21,23,25]
torso = [27,28,29]
left_hand = [32,34,36]
right_hand = [31,33,35]

def getch():
    return sys.stdin.read(1)


#gets called by the service and will read the current, voltage and temperature from their correspodning register values
def get_present_motor_sensors(req):
    print("get_motor_sensors called")
    # if(req.id in left_arm):
    #       dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_CURRENT)
    #       dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_VOLTAGE)
    #       dxl_present_temp, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_TEMP)
    if(req.id in left_arm or head or torso):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_CURRENT)
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_VOLTAGE)
        dxl_present_temp, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_TEMP)
    if(req.id in right_arm):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler2, req.id, ADDR_PRESENT_CURRENT)
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler2, req.id, ADDR_PRESENT_VOLTAGE)
        dxl_present_temp, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler2, req.id, ADDR_PRESENT_TEMP)
    if(req.id in left_leg):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler3, req.id, ADDR_PRESENT_CURRENT)
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler3, req.id, ADDR_PRESENT_VOLTAGE)
        dxl_present_temp, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler3, req.id, ADDR_PRESENT_TEMP)
    if(req.id in right_leg):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler4, req.id, ADDR_PRESENT_CURRENT)
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler4, req.id, ADDR_PRESENT_VOLTAGE)
        dxl_present_temp, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler4, req.id, ADDR_PRESENT_TEMP)
    if(req.id in left_hand):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_CURRENT_HANDS)
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_VOLTAGE_HANDS)
        dxl_present_temp, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(portHandler1, req.id, ADDR_PRESENT_TEMP_HANDS)    
    if(req.id in right_hand):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(portHandler2, req.id, ADDR_PRESENT_CURRENT_HANDS)
        dxl_present_voltage, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(portHandler2, req.id, ADDR_PRESENT_VOLTAGE_HANDS)
        dxl_present_temp, dxl_comm_result, dxl_error = packetHandler2.read2ByteTxRx(portHandler2, req.id, ADDR_PRESENT_TEMP_HANDS)
    return GetMotorSensorsResponse(dxl_present_current, dxl_present_voltage, dxl_present_temp) #


#initialize the service for the motor sensors
def init_motor_sensors_service():
    s = rospy.Service('get_motor_sensors', GetMotorSensors, get_present_motor_sensors) #rospy.seriver(topic, message type, function which gets called)
    print("Ready to get get motor sensors")
    rospy.spin() #keeps ROS from ending 

if __name__ == "__main__":
    # Open port
    try:
       print(rospy.get_param_names())
    except ROSException:
       print("could not get param name")
    try:
       portHandler1.openPort()
       portHandler2.openPort()
    #    portHandler3.openPort()
    #    portHandler4.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler1.setBaudRate(BAUDRATE)
        portHandler2.setBaudRate(BAUDRATE)
        # portHandler3.setBaudRate(BAUDRATE)
        # portHandler4.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    # Initialize node
    rospy.init_node('motor_sensor_server')
    # Initialize motor sensors service
    init_motor_sensors_service()
    

