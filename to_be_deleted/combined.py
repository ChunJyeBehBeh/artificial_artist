#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import ctypes
import os
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import numpy as np
import ik as ik
import time
from draw import *
import numpy as np


filename = "Love.png"
drawer = Drawer(filename)
drawer.findPath()
    
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

# Program Parameters
print_param = True

def user_input(id):
    val = float(input(
        "Enter Goal Position for Motor {} in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position):".format(
            id)))
    return val, int(val / 300 * 1023)


def program_input(value):
    value = float(value)
    return value, int(value / 300 * 1023)


class Dynamixel():
    def __init__(self, args):
        # Parameters from Terminal
        self.user_input = args.user_input

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
        self.ADDR_MX_GOAL_POSITION = 30  # <-----
        self.ADDR_MX_PRESENT_POSITION = 36
        self.ADDR_AX12A_MOVE_SPEED = 32

        # Data Byte Length
        self.LEN_MX_GOAL_POSITION = 4
        self.LEN_MX_PRESENT_POSITION = 2

        # Protocol version
        self.PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID_6 = 6  # Dynamixel ID : 6
        self.DXL_ID_1 = 1  # Dynamixel ID : 1
        self.DXL_ID_2 = 2  # Dynamixel ID : 2
        self.DXL_ID_3 = 3  # Dynamixel ID : 3
        self.BAUDRATE = 1000000  # Dynamixel default baudrate : 57600
        self.DEVICENAME = "/dev/ttyUSB0"  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold
        self.dxl_addparam_result = 0
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize Groupsyncwrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler,
                                             self.ADDR_MX_GOAL_POSITION, self.LEN_MX_GOAL_POSITION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
    
    def read_speed(self,id,arg):
        '''
        arg = True for debug purpose, then it will print out the msg
        '''
        speed, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, id , self.ADDR_AX12A_MOVE_SPEED)
        if arg:
            print("Speed of ID {} is {}.".format(id,speed))
        return int(speed)

    # Function to read present position
    def read_pos(self, id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id,
                                                                                            self.ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            # [TxRxResult] Incorrect status packet!
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("here!!!")
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return dxl_present_position, dxl_comm_result, dxl_error

    # Set servo move speed
    def set_joint_speed(self, id, speed):
        # print("DataType of speed: ",type(speed))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_AX12A_MOVE_SPEED,
                                                                       int(speed))
        # print("Set Speed ",id,'---',dxl_comm_result,'---',dxl_error)
        if dxl_comm_result != COMM_SUCCESS:
            # print("fail")
            return self.set_joint_speed(id, speed)
        elif dxl_error != 0:
            # print("fail")
            return self.set_joint_speed(id, speed)
        # elif self.read_speed(id, False)!= int(speed):
        #     self.set_joint_speed(id,speed)
        else:
            print("Dynamixel#%d speed has been set: %s" % (id, speed))

    def enable_servo_torque(self, id):
        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_MX_TORQUE_ENABLE,
                                                                       self.TORQUE_ENABLE)
        # print('Enable Torque ',id,'---',dxl_comm_result,'---',dxl_error)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % id)

    def disable_servo_torque(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_MX_TORQUE_ENABLE,
                                                                       self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def syncwrite_storage(self, id, goal_position):
        # Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = ctypes.c_ubyte(
            self.groupSyncWrite.addParam(id, goal_position)).value
        if dxl_addparam_result != 1:
            print(dxl_addparam_result)
            print("[ID:%03d] groupSyncWrite addparam failed" % (id))
            quit()

    def goal_send(self,id,position):
        _, _ = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_MX_GOAL_POSITION, position)

    def print_status(self, id, goal_position, present_position):
        print("[ID: {} GoalPos: {:.03f}  PresPos: {:.03f} ]".format(id, goal_position, present_position))

def move_to():
    '''
    Currently this function is uesless. Can just ignore it
    '''
    servo.goal_send(servo.DXL_ID_3,GoalPosition_3)
    servo.goal_send(servo.DXL_ID_6,GoalPosition_6)
    servo.goal_send(servo.DXL_ID_2,GoalPosition_2)
    servo.goal_send(servo.DXL_ID_1,GoalPosition_1)

    while 1:
        
        # Read present position
        dxl_present_position_6, _, _ = servo.read_pos(servo.DXL_ID_6)
        dxl_present_position_1, _, _ = servo.read_pos(servo.DXL_ID_1)
        dxl_present_position_2, _, _ = servo.read_pos(servo.DXL_ID_2)
        dxl_present_position_3, _, _ = servo.read_pos(servo.DXL_ID_3)

        dxl_present_position_6_deg = dxl_present_position_6 / 1023.0 * 300.0
        dxl_present_position_1_deg = dxl_present_position_1 / 1023.0 * 300.0
        dxl_present_position_2_deg = dxl_present_position_2 / 1023.0 * 300.0
        dxl_present_position_3_deg = dxl_present_position_3 / 1023.0 * 300.0

        # Print Statues
        if print_param:
            servo.print_status(servo.DXL_ID_3, GoalPosition_3_deg, dxl_present_position_3_deg)
            servo.print_status(servo.DXL_ID_6, GoalPosition_6_deg, dxl_present_position_6_deg)
            servo.print_status(servo.DXL_ID_2, GoalPosition_2_deg, dxl_present_position_2_deg)
            servo.print_status(servo.DXL_ID_1, GoalPosition_1_deg, dxl_present_position_1_deg)

        if ((abs(GoalPosition_6 - dxl_present_position_6) <= servo.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(GoalPosition_1 - dxl_present_position_1) <= servo.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(GoalPosition_2 - dxl_present_position_2) <= servo.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(GoalPosition_3 - dxl_present_position_3) <= servo.DXL_MOVING_STATUS_THRESHOLD)):
                break

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dynamixel Servo Control')
    parser.add_argument('--user_input', action='store_false', help='ignore this argument in order to ask goal from program')
    args = parser.parse_args()

    servo = Dynamixel(args)

    # Enable Dynamixel Torque
    servo.enable_servo_torque(servo.DXL_ID_6)
    servo.enable_servo_torque(servo.DXL_ID_1)
    servo.enable_servo_torque(servo.DXL_ID_2)
    servo.enable_servo_torque(servo.DXL_ID_3)

    # Read Pos of Servo
    dxl6_present_position, _, _ = servo.read_pos(servo.DXL_ID_6)
    dxl1_present_position, _, _ = servo.read_pos(servo.DXL_ID_1)
    dxl2_present_position, _, _ = servo.read_pos(servo.DXL_ID_2)
    dxl3_present_position, _, _ = servo.read_pos(servo.DXL_ID_3)

    print("Set speed haven't done")
    servo.read_speed(servo.DXL_ID_6,True)
    servo.read_speed(servo.DXL_ID_1,True)
    servo.read_speed(servo.DXL_ID_2,True)
    servo.read_speed(servo.DXL_ID_3,True)

    # Set Speed of Servo
    joint_speed = 50
    servo.set_joint_speed(servo.DXL_ID_6, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_3, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_2, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_1, joint_speed)
    
    print("Set speed done")
    servo.read_speed(servo.DXL_ID_6,True)
    servo.read_speed(servo.DXL_ID_1,True)
    servo.read_speed(servo.DXL_ID_2,True)
    servo.read_speed(servo.DXL_ID_3,True)

    while 1:
        if args.user_input:
            # User input goal position
            GoalPosition_3_deg, GoalPosition_3 = user_input(3)
            GoalPosition_6_deg, GoalPosition_6 = user_input(6)
            GoalPosition_2_deg, GoalPosition_2 = user_input(2)
            GoalPosition_1_deg, GoalPosition_1 = user_input(1)
            
            move_to()

        else:
            '''
            Get list from image_processing, if x=9999=9999,that means we need to lift up/down our marker [transition state], set transition = !transition, lift up
            Move the marker to the first point of next line, that set transition = !transition, lift down
            Draw Draw Draw until x=y=0 again, set transition = !transition, lift up

            recursive until last packet/ line.....

            Report "Finish Drawing" to user 
            '''
            # arr =drawer.draw()
            # arr=np.asarray(arr)*0.025
            # arr=arr[::4]
            # arr = arr.tolist()
            arr = [[5,0],[7,0],[7,3],[5,0]]
            # arr=np.asarray(arr)*0.05
            # arr = arr[::20]
  
            for i in arr:
                print("From Drawing: ",i[0]+10-4, i[1],2+4)
                arr = ik.get_inverse(i[0]+10-4, i[1],-1.2+4)                # offset for end effector

                arr[3] = -arr[3]
                arr = [i + 150.0 for i in arr]
                print("From IK(after +150): ",arr)
                
                GoalPosition_3_deg, GoalPosition_3 = program_input(arr[0])
                GoalPosition_6_deg, GoalPosition_6 = program_input(arr[1])
                GoalPosition_2_deg, GoalPosition_2 = program_input(arr[2])
                GoalPosition_1_deg, GoalPosition_1 = program_input(arr[3])

                # angle into param_goal_position_6 is degree/300*1023
                move_to()
                print("--- Delay ---")
                time.sleep(1.0)
                print("--- Next ---")

                # break

    # Disable Dynamixel Torque
    servo.disable_servo_torque(servo.DXL_ID_6)
    servo.disable_servo_torque(servo.DXL_ID_1)
    servo.disable_servo_torque(servo.DXL_ID_2)
    servo.disable_servo_torque(servo.DXL_ID_3)

    # Close port
    servo.portHandler.closePort()
