 #!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import warnings
import ctypes
import os
import numpy as np
import time
import argparse
import ctypes
from math import sqrt
import ik as ik
from draw import *
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from play_sound import *

# Parameters for Program Drawing
H_move =  2+4
H_draw = -2.7+4
filename = "Image/circle1.png"
# filename = "Image/result.jpg"

drawer = Drawer(filename,H_draw,H_move,False)
drawer.findPath()

# Program Parameters for Dynamixel Servos
print_param = True          # printing status of Dynamixel Servos
testing = False
min_X = 10
max_X = 30
min_Y =-15
max_Y = 15

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

def user_input(id):
    val = float(input(
        "Enter Goal Position for Motor {} in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position):".format(
            id)))
    return val, int(val / 300 * 1023)

def get_offset(val):
    if 9<=val<=11:
        offset = (-2*val) +19.5
    if 11<val<=14:
        offset = (-val/1.3) +7 
    else:
        offset = 0
    return offset
def program_input(value):
    value = float(value)
    return value, int(value / 300 * 1023)


class Dynamixel():
    def __init__(self, args):
        # Parameters from Terminal
        self.user_input = args.user_input

        # Control table address
        self.ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
        self.ADDR_MX_GOAL_POSITION = 30  
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
        self.DEVICENAME = "/dev/ttyUSB0"  
        # Check which port is being used on your controller
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

def move_to(status_moving):
    # Allocate goal position value into byte array
    param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_6)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_6)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_6))]
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_1)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_1)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_1))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_2)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_2)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_2))]
    param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_3)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_3)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_3))]
    if status_moving:
        print("Moving the arm")
        servo.goal_send(servo.DXL_ID_6, param_goal_position_6)
        servo.goal_send(servo.DXL_ID_3, param_goal_position_3)
        servo.goal_send(servo.DXL_ID_1, param_goal_position_1)
        servo.goal_send(servo.DXL_ID_2, param_goal_position_2)
    else:
        # Send goal to syncwrite storage
        servo.syncwrite_storage(servo.DXL_ID_6, param_goal_position_6)
        servo.syncwrite_storage(servo.DXL_ID_1, param_goal_position_1)
        servo.syncwrite_storage(servo.DXL_ID_2, param_goal_position_2)
        servo.syncwrite_storage(servo.DXL_ID_3, param_goal_position_3)

        # Syncwrite goal position
        dxl_comm_result = servo.groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % servo.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        servo.groupSyncWrite.clearParam()

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

    # Set Speed of Servo
    joint_speed = 50
    servo.set_joint_speed(servo.DXL_ID_6, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_3, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_2, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_1, joint_speed)

    # Go to home position 
    servo.goal_send(servo.DXL_ID_6, 512)
    servo.goal_send(servo.DXL_ID_3, 512)
    servo.goal_send(servo.DXL_ID_1, 512)
    servo.goal_send(servo.DXL_ID_2, 512)

    while 1:
        if args.user_input:
            # User input goal position
            GoalPosition_3_deg, GoalPosition_3 = user_input(3)
            GoalPosition_6_deg, GoalPosition_6 = user_input(6)
            GoalPosition_2_deg, GoalPosition_2 = user_input(2)
            GoalPosition_1_deg, GoalPosition_1 = user_input(1)
            
            move_to(True)       # Not syncwrite

        else:            
            if not testing:
                # Drawing from input image
                arr = drawer.draw()
                print("Number of point to IK: {}".format(len(arr)))

                # Factor x coordinate & y coordinate
                for i in arr:
                    i[0] = i[0]*0.05
                    i[1] = i[1]*0.05

                # arr = np.round(arr,1)             # skip every 10 numbers
                # _,idx = np.unique(arr[:,0], axis=0, return_index=True)
                # arr = arr[np.sort(idx)]
                # print("After Filtered, Number of point to IK: {}".format(len(arr)))
            else:
                print("Testing")
                arr = [[10,0,0],[11,0,0],[12,0,0],[13,0,0],[14,0,0],[15,0,0]]

            for index, i in enumerate(arr):
                '''
                Y-coordinate from image processing is always positive, 
                "-15" offset to use second quadrant
                '''
                if(int(i[2])==H_move):
                        status_move = True

                if not testing:
                    # Drawing from input image
                    x_coor = int(i[0])+10
                    y_coor = int(i[1])-15
                    print("From Drawing for point {}/{}:".format(index+1,len(arr)),i[0]+10-4, i[1]-15,i[2])
                    arr = ik.get_inverse(i[0]+10, i[1]-15,i[2])                # offset for end effector

                else:
                    x_coor = int(i[0])
                    y_coor = int(i[1])

                    print("From Drawing: ",i[0], i[1],i[2])
                    print("if-else value:",sqrt(i[0]**2 + i[1]**2) )
                    val = sqrt(i[0]**2 + i[1]**2)
                    offset = get_offset(val)
                    print("val: {0} offset:{1}".format(val,offset))
                    
                    arr = ik.get_inverse(i[0]+20, i[1],(i[2]))                # offset for end effector

                arr[3] = -arr[3]
                arr = [i + 150.0 for i in arr]
                print("From IK(after +150): ",arr)
                
                GoalPosition_3_deg, GoalPosition_3 = program_input(arr[0])
                GoalPosition_6_deg, GoalPosition_6 = program_input(arr[1])
                GoalPosition_2_deg, GoalPosition_2 = program_input(arr[2])
                GoalPosition_1_deg, GoalPosition_1 = program_input(arr[3])

                if ((x_coor>min_X or x_coor<max_X) and (y_coor>min_Y or y_coor<max_Y)):
                    move_to(status_move)
                else:
                    warnings.warn('Exceed the Limit. Skip that point')

                print("--- Wait ---")
                time.sleep(1.0)
                print("--- Next ---")

            break

    print("Finished")
    play_sound()
    
    # Disable Dynamixel Torque
    servo.disable_servo_torque(servo.DXL_ID_6)
    servo.disable_servo_torque(servo.DXL_ID_1)
    servo.disable_servo_torque(servo.DXL_ID_2)
    servo.disable_servo_torque(servo.DXL_ID_3)

    # Close port
    servo.portHandler.closePort()
