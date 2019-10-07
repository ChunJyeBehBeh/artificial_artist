#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
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
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_6                    = 6                 # Dynamixel ID : 6
DXL_ID_1                    = 1                 # Dynamixel ID : 1
DXL_ID_2                    = 2                 # Dynamixel ID : 2
DXL_ID_3                    = 3                 # Dynamixel ID : 3
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = "COM3"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result_6, dxl_error_6 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result_1, dxl_error_1 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result_2, dxl_error_2 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result_3, dxl_error_3 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if (dxl_comm_result_6 or dxl_comm_result_1 or dxl_comm_result_2 or dxl_comm_result_3) != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result_6))
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result_1))
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result_2))
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result_3))
elif (dxl_error_6 | dxl_error_1 | dxl_error_2 | dxl_error_3) != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error_6))
    print("%s" % packetHandler.getRxPacketError(dxl_error_1))
    print("%s" % packetHandler.getRxPacketError(dxl_error_2))
    print("%s" % packetHandler.getRxPacketError(dxl_error_3))
else:
    print("Dynamixel has been successfully connected")

while 1:
    # print("Press any key to continue! (or press ESC to quit!)")
    # if getch() == chr(0x1b):
    #     break

    GoalPosition_6 = input("Enter Goal Position for Motor 6 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    GoalPosition_6_deg = float(GoalPosition_6)
    GoalPosition_6 = int(GoalPosition_6_deg/300*1023)

    GoalPosition_1 = input("Enter Goal Position for Motor 1 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    GoalPosition_1_deg = float(GoalPosition_1)
    GoalPosition_1 = int(GoalPosition_1_deg/300*1023)

    GoalPosition_2 = input("Enter Goal Position for Motor 2 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    GoalPosition_2_deg = float(GoalPosition_2)
    GoalPosition_2 = int(GoalPosition_2_deg/300*1023)

    GoalPosition_3 = input("Enter Goal Position for Motor 3 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    GoalPosition_3_deg = float(GoalPosition_3)
    GoalPosition_3 = int(GoalPosition_3_deg/300*1023)

    ''' Array section'''
    
    
    # Write goal position
    # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    dxl_comm_result_6, dxl_error_6 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_GOAL_POSITION, GoalPosition_6)
    dxl_comm_result_1, dxl_error_1 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_GOAL_POSITION, GoalPosition_1)
    dxl_comm_result_2, dxl_error_2 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_GOAL_POSITION, GoalPosition_2)
    dxl_comm_result_3, dxl_error_3 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_GOAL_POSITION, GoalPosition_3)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position_6, dxl_comm_result_6, dxl_error_6 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_PRESENT_POSITION)
        dxl_present_position_1, dxl_comm_result_1, dxl_error_1 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_PRESENT_POSITION)
        dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_PRESENT_POSITION)
        dxl_present_position_3, dxl_comm_result_3, dxl_error_3 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_PRESENT_POSITION)
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # elif dxl_error != 0:
        #     print("%s" % packetHandler.getRxPacketError(dxl_error))

        dxl_present_position_6_deg = dxl_present_position_6/1023*300
        dxl_present_position_1_deg = dxl_present_position_1/1023*300
        dxl_present_position_2_deg = dxl_present_position_2/1023*300
        dxl_present_position_3_deg = dxl_present_position_3/1023*300

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d  [ID:%03d] GoalPos:%03d  PresPos:%03d  [ID:%03d] GoalPos:%03d  PresPos:%03d  [ID:%03d] GoalPos:%03d  PresPos:%03d"
              % (DXL_ID_6, GoalPosition_6_deg, dxl_present_position_6_deg, DXL_ID_1, GoalPosition_1_deg, dxl_present_position_1_deg,
                 DXL_ID_2, GoalPosition_2_deg, dxl_present_position_2_deg, DXL_ID_3, GoalPosition_3_deg, dxl_present_position_3_deg))

        if not (abs(GoalPosition_6 - dxl_present_position_6) or abs(GoalPosition_1 - dxl_present_position_1) or
                abs(GoalPosition_2 - dxl_present_position_2) or abs(GoalPosition_3 - dxl_present_position_3)) > DXL_MOVING_STATUS_THRESHOLD:
            break


# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
