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

from dynamixel_sdk import *  # Uses Dynamixel SDK library
import os
import ctypes

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Function to read present position
def read_pos(id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    return dxl_present_position

# Set move speed
def set_joint_speed(id,speed):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, id, ADDR_AX12A_MOVE_SPEED, speed)
    if dxl_comm_result != COMM_SUCCESS:
        return set_joint_speed(id,speed)
    elif dxl_error != 0:
        return set_joint_speed(id,speed)
    else:
        print("Dynamixel#%d speed has been set: %s" % (id,speed))
        return 1

# Control table address
ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION = 30                  # <-----
ADDR_MX_PRESENT_POSITION = 36
ADDR_AX12A_MOVE_SPEED = 32

# Data Byte Length
LEN_MX_GOAL_POSITION = 4
LEN_MX_PRESENT_POSITION = 2

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_6 = 6  # Dynamixel ID : 6
DXL_ID_1 = 1  # Dynamixel ID : 1
DXL_ID_2 = 2  # Dynamixel ID : 2
DXL_ID_3 = 3  # Dynamixel ID : 3
BAUDRATE = 1000000  # Dynamixel default baudrate : 57600
DEVICENAME = "/dev/ttyUSB0"  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold
dxl_addparam_result = 0
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize Groupsyncwrite instance
group_num = GroupSyncWrite(portHandler, packetHandler,
                           ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

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
dxl_comm_result_6, dxl_error_6 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_TORQUE_ENABLE,
                                                              TORQUE_ENABLE)
dxl_comm_result_1, dxl_error_1 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_TORQUE_ENABLE,
                                                              TORQUE_ENABLE)
dxl_comm_result_2, dxl_error_2 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_TORQUE_ENABLE,
                                                              TORQUE_ENABLE)
dxl_comm_result_3, dxl_error_3 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_TORQUE_ENABLE,
                                                              TORQUE_ENABLE)
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
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    dxl6_present_position = read_pos(DXL_ID_6)
    dxl1_present_position = read_pos(DXL_ID_1)
    dxl2_present_position = read_pos(DXL_ID_2)
    dxl3_present_position = read_pos(DXL_ID_3)

    joint_speed = 1
    set_joint_speed(DXL_ID_6,joint_speed)
    set_joint_speed(DXL_ID_1,joint_speed)
    set_joint_speed(DXL_ID_2,joint_speed)
    set_joint_speed(DXL_ID_3,joint_speed)

    # GoalPosition_6 = input(
    #     "Enter Goal Position for Motor 6 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    # GoalPosition_6_deg = float(GoalPosition_6)
    # GoalPosition_6 = int(GoalPosition_6_deg / 300 * 1023)

    # GoalPosition_1 = input(
    #     "Enter Goal Position for Motor 1 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    # GoalPosition_1_deg = float(GoalPosition_1)
    # GoalPosition_1 = int(GoalPosition_1_deg / 300 * 1023)

    # GoalPosition_2 = input(
    #     "Enter Goal Position for Motor 2 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    # GoalPosition_2_deg = float(GoalPosition_2)
    # GoalPosition_2 = int(GoalPosition_2_deg / 300 * 1023)

    # GoalPosition_3 = input(
    #     "Enter Goal Position for Motor 3 in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position): ")
    # GoalPosition_3_deg = float(GoalPosition_3)
    # GoalPosition_3 = int(GoalPosition_3_deg / 300 * 1023)

    '''
    Write goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
    -------------------------------------------------------------------------------------------------------------------------------
    dxl_comm_result_6, dxl_error_6 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_GOAL_POSITION,
                                                                  GoalPosition_6)
    dxl_comm_result_1, dxl_error_1 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_GOAL_POSITION,
                                                                  GoalPosition_1)
    dxl_comm_result_2, dxl_error_2 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_GOAL_POSITION,
                                                                  GoalPosition_2)
    dxl_comm_result_3, dxl_error_3 = packetHandler.write4ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_GOAL_POSITION,
                                                                  GoalPosition_3)
    '''
    # Input goal position
    GoalPosition_6 = int(input("Motor 6 Goal pos?"))
    GoalPosition_1 = int(input("Motor 1 Goal pos?"))
    GoalPosition_2 = int(input("Motor 2 Goal pos?"))
    GoalPosition_3 = int(input("Motor 3 Goal pos?"))

    # Allocate goal position value into byte array
    param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_6)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_6)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_6))]
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_1)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_1)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_1))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_2)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_2)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_2))]
    param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_3)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_3)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_3))]

    # Add Dynamixel#n goal position value to the Syncwrite storage
    dxl_addparam_result = ctypes.c_ubyte(
        group_num.addParam(DXL_ID_1, param_goal_position_1)).value
    print(dxl_addparam_result)
    if dxl_addparam_result != 1:
        print(dxl_addparam_result)
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_1))
        quit()
        
    dxl_addparam_result = ctypes.c_ubyte(
        group_num.addParam(DXL_ID_6, param_goal_position_6)).value
    print(dxl_addparam_result)
    if dxl_addparam_result != 1:
        print(dxl_addparam_result)
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_6))
        quit()
    dxl_addparam_result = ctypes.c_ubyte(
        group_num.addParam(DXL_ID_2, param_goal_position_2)).value
    print(dxl_addparam_result)
    if dxl_addparam_result != 1:
        print(dxl_addparam_result)
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_2))
        quit()
    dxl_addparam_result = ctypes.c_ubyte(
        group_num.addParam(DXL_ID_3, param_goal_position_3)).value
    print(dxl_addparam_result)
    if dxl_addparam_result != 1:
        print(dxl_addparam_result)
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_3))
        quit()
    # dxl_addparam_result = ctypes.c_ubyte(
    #     group_num.addParam(DXL_ID_1, [GoalPosition_1])).value
    # print(dxl_addparam_result)
    # if dxl_addparam_result != 1:
    #     print(dxl_addparam_result)
    #     print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_1))
    #     quit()
        
    # dxl_addparam_result = ctypes.c_ubyte(
    #     group_num.addParam(DXL_ID_6, [GoalPosition_6])).value
    # print(dxl_addparam_result)
    # if dxl_addparam_result != 1:
    #     print(dxl_addparam_result)
    #     print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_6))
    #     quit()
    # dxl_addparam_result = ctypes.c_ubyte(
    #     group_num.addParam(DXL_ID_2, [GoalPosition_2])).value
    # print(dxl_addparam_result)
    # if dxl_addparam_result != 1:
    #     print(dxl_addparam_result)
    #     print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_2))
    #     quit()
    # dxl_addparam_result = ctypes.c_ubyte(
    #     group_num.addParam(DXL_ID_3, [GoalPosition_3])).value
    # print(dxl_addparam_result)
    # if dxl_addparam_result != 1:
    #     print(dxl_addparam_result)
    #     print("[ID:%03d] groupSyncWrite addparam failed" % (DXL_ID_3))
    #     quit()
    print("----------------")
    print(group_num.data_dict)

    # Syncwrite goal position
    dxl_comm_result = group_num.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    group_num.clearParam()

    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position_6, dxl_comm_result_6, dxl_error_6 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_6,
                                                                                             ADDR_MX_PRESENT_POSITION)
        dxl_present_position_1, dxl_comm_result_1, dxl_error_1 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_1,
                                                                                             ADDR_MX_PRESENT_POSITION)
        dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_2,
                                                                                             ADDR_MX_PRESENT_POSITION)
        dxl_present_position_3, dxl_comm_result_3, dxl_error_3 = packetHandler.read4ByteTxRx(portHandler, DXL_ID_3,
                                                                                             ADDR_MX_PRESENT_POSITION)
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # elif dxl_error != 0:
        #     print("%s" % packetHandler.getRxPacketError(dxl_error))

        dxl_present_position_6_deg = dxl_present_position_6 / 1023 * 300
        dxl_present_position_1_deg = dxl_present_position_1 / 1023 * 300
        dxl_present_position_2_deg = dxl_present_position_2 / 1023 * 300
        dxl_present_position_3_deg = dxl_present_position_3 / 1023 * 300

        print(
            "[ID:%03d] GoalPos:%03d  PresPos:%03d  [ID:%03d] GoalPos:%03d  PresPos:%03d  [ID:%03d] GoalPos:%03d  PresPos:%03d  [ID:%03d] GoalPos:%03d  PresPos:%03d"
            % (DXL_ID_6, GoalPosition_6_deg, dxl_present_position_6_deg, DXL_ID_1, GoalPosition_1_deg,
               dxl_present_position_1_deg,
               DXL_ID_2, GoalPosition_2_deg, dxl_present_position_2_deg, DXL_ID_3, GoalPosition_3_deg,
               dxl_present_position_3_deg))

        if not (abs(GoalPosition_6 - dxl_present_position_6) or abs(GoalPosition_1 - dxl_present_position_1) or
                abs(GoalPosition_2 - dxl_present_position_2) or abs(
                    GoalPosition_3 - dxl_present_position_3)) > DXL_MOVING_STATUS_THRESHOLD:
            break

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
