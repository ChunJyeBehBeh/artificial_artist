#!/usr/bin/env python
# -*- coding: utf-8 -*-
# test multi motor sync write function

import os
import time

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

from dynamixel_sdk import *                     # Uses Dynamixel SDK library

# Data Byte Length
LEN_AX12A_GOAL_POSITION       = 4
LEN_AX12A_PRESENT_POSITION    = 4

# Control table address
ADDR_AX12A_TORQUE_ENABLE      = 24                 # Control table address is different in Dynamixel model
ADDR_AX12A_GOAL_POSITION      = 30
ADDR_AX12A_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1                 # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 4                 # Dynamixel ID : 1
DXL2_ID                      = 5                 # Dynamixel ID : 1

BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'            # Check which port is being used on your controller

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX12A_GOAL_POSITION, LEN_AX12A_GOAL_POSITION)

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

# Function to Enabling/Disabling torque
def torque_enable(id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX12A_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % id)

def torque_disable(id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX12A_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Enable Dynamixels Torque
torque_enable(DXL1_ID)
torque_enable(DXL2_ID)
time.sleep(1.0)

# Function to setup parameters
def add_params(id,params):
    dxl_addparam_result = groupSyncWrite.addParam(id, params)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % id)
        quit()

# Function to read present position
def read_pos(id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_AX12A_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    return dxl_present_position

while 1:
    # Read Dynamixel#1 present position
    # Read Dynamixel#2 present position
    dxl1_present_position = read_pos(DXL1_ID)
    dxl2_present_position = read_pos(DXL2_ID)

    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))

    # Input goal position
    dxl1_goal_position = int(input("Motor 1 Goal pos?"))
    dxl2_goal_position = int(input("Motor 2 Goal pos?"))

    # Allocate goal position value into byte array
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    add_params(DXL1_ID, param_goal_position_1)

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    add_params(DXL2_ID, param_goal_position_2)
    
    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    
    while 1:
        dxl1_present_position = read_pos(DXL1_ID)
        dxl2_present_position = read_pos(DXL2_ID)
        print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position))
        if ((abs(dxl1_goal_position - dxl1_present_position) < DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl2_goal_position - dxl2_present_position) < DXL_MOVING_STATUS_THRESHOLD)):
            break
    print("Done")

# Disable Dynamixel Torque
torque_disable(DXL1_ID)
torque_disable(DXL2_ID)

# Close port
portHandler.closePort()
