#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import ctypes
import os
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import ik as ik

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
        speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID_1, self.ADDR_AX12A_MOVE_SPEED)
        if arg:
            print("Speed of ID {} is {}.".format(id,speed))
        return speed

    # Function to read present position
    def read_pos(self, id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id,
                                                                                            self.ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
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
        # elif self.read_speed(id, False)!= speed:
        #     self.set_joint_speed(id,speed)
        else:
            print("Dynamixel#%d speed has been set: %s" % (id, speed))
            return 1

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
        print(dxl_addparam_result)
        if dxl_addparam_result != 1:
            print(dxl_addparam_result)
            print("[ID:%03d] groupSyncWrite addparam failed" % (id))
            quit()

    def print_status(self, id, goal_position, present_position):
        print("[ID: {} GoalPos: {}  PresPos: {} ]".format(id, goal_position, present_position))

def test():
    '''
    Currently this function is uesless. Can just ignore it
    '''
    # Allocate goal position value into byte array
    # param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(GoalPosition_6))]
    # param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(GoalPosition_1))]
    # param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(GoalPosition_2))]
    # param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(GoalPosition_3))]

    param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_6)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_6)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_6))]
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_1)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_1)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_1))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_2)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_2)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_2))]
    param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_3)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_3)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_3))]

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

    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position_6, dxl_comm_result_6, dxl_error_6 = servo.read_pos(servo.DXL_ID_6)
        dxl_present_position_1, dxl_comm_result_1, dxl_error_1 = servo.read_pos(servo.DXL_ID_1)
        dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = servo.read_pos(servo.DXL_ID_2)
        dxl_present_position_3, dxl_comm_result_3, dxl_error_3 = servo.read_pos(servo.DXL_ID_3)

        dxl_present_position_6_deg = dxl_present_position_6 / 1023 * 300
        dxl_present_position_1_deg = dxl_present_position_1 / 1023 * 300
        dxl_present_position_2_deg = dxl_present_position_2 / 1023 * 300
        dxl_present_position_3_deg = dxl_present_position_3 / 1023 * 300

        # Print Statues
        servo.print_status(servo.DXL_ID_6, GoalPosition_6_deg, dxl_present_position_6_deg)
        servo.print_status(servo.DXL_ID_1, GoalPosition_1_deg, dxl_present_position_1_deg)
        servo.print_status(servo.DXL_ID_2, GoalPosition_2_deg, dxl_present_position_2_deg)
        servo.print_status(servo.DXL_ID_3, GoalPosition_3_deg, dxl_present_position_3_deg)

        if not (abs(GoalPosition_6 - dxl_present_position_6) or abs(GoalPosition_1 - dxl_present_position_1) or
                abs(GoalPosition_2 - dxl_present_position_2) or abs(
                    GoalPosition_3 - dxl_present_position_3)) > servo.DXL_MOVING_STATUS_THRESHOLD:
            break
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dynamixel Servo Control')
    parser.add_argument('--user_input', action='store_false', help='ignore this argument in order to ask goal from program')
    args = parser.parse_args()

    servo = Dynamixel(args)
    if args.user_input == True:
        print("True")
    else:
        print("False")  

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
    joint_speed = 10
    servo.set_joint_speed(servo.DXL_ID_6, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_1, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_2, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_3, joint_speed)

    
    print("Set speed done")
    servo.read_speed(servo.DXL_ID_6,True)
    servo.read_speed(servo.DXL_ID_1,True)
    servo.read_speed(servo.DXL_ID_2,True)
    servo.read_speed(servo.DXL_ID_3,True)

    x= 20
    while 1:
        if args.user_input:
            # User input goal position
            GoalPosition_3_deg, GoalPosition_3 = user_input(3)
            GoalPosition_6_deg, GoalPosition_6 = user_input(6)
            GoalPosition_2_deg, GoalPosition_2 = user_input(2)
            GoalPosition_1_deg, GoalPosition_1 = user_input(1)
            
            # Allocate goal position value into byte array
            param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(
                GoalPosition_6)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_6)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_6))]
            param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(
                GoalPosition_1)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_1)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_1))]
            param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(
                GoalPosition_2)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_2)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_2))]
            param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(
                GoalPosition_3)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_3)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_3))]

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

            # if dxl_comm_result != COMM_SUCCESS:
            #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            # elif dxl_error != 0:
            #     print("%s" % packetHandler.getRxPacketError(dxl_error))

            while 1:
                # Read present position
                dxl_present_position_6, dxl_comm_result_6, dxl_error_6 = servo.read_pos(servo.DXL_ID_6)
                dxl_present_position_1, dxl_comm_result_1, dxl_error_1 = servo.read_pos(servo.DXL_ID_1)
                dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = servo.read_pos(servo.DXL_ID_2)
                dxl_present_position_3, dxl_comm_result_3, dxl_error_3 = servo.read_pos(servo.DXL_ID_3)

                dxl_present_position_6_deg = dxl_present_position_6 / 1023 * 300
                dxl_present_position_1_deg = dxl_present_position_1 / 1023 * 300
                dxl_present_position_2_deg = dxl_present_position_2 / 1023 * 300
                dxl_present_position_3_deg = dxl_present_position_3 / 1023 * 300

                # Print Statues
                servo.print_status(servo.DXL_ID_6, GoalPosition_6_deg, dxl_present_position_6_deg)
                servo.print_status(servo.DXL_ID_1, GoalPosition_1_deg, dxl_present_position_1_deg)
                servo.print_status(servo.DXL_ID_2, GoalPosition_2_deg, dxl_present_position_2_deg)
                servo.print_status(servo.DXL_ID_3, GoalPosition_3_deg, dxl_present_position_3_deg)

                if not (abs(GoalPosition_6 - dxl_present_position_6) or abs(GoalPosition_1 - dxl_present_position_1) or
                        abs(GoalPosition_2 - dxl_present_position_2) or abs(
                            GoalPosition_3 - dxl_present_position_3)) > servo.DXL_MOVING_STATUS_THRESHOLD:
                    break
        else:
            for i in range(0,x,2):
                # arr = [150, 170, 150, 150]
                # arr = ik.get_inverse(18.7+14.5, 0.0, 16.5-14.5)           # home position
                arr = ik.get_inverse(18.7, 0, 16.5+5)
                arr = [i + 150.0 for i in arr]
                # arr[1]=arr[1]-60
                print(arr)

                GoalPosition_3_deg, GoalPosition_3 = program_input(arr[0])
                GoalPosition_6_deg, GoalPosition_6 = program_input(arr[1])
                GoalPosition_2_deg, GoalPosition_2 = program_input(arr[2])
                GoalPosition_1_deg, GoalPosition_1 = program_input(arr[3])

                # Allocate goal position value into byte array
                param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(
                    GoalPosition_6)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_6)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_6))]
                param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(
                    GoalPosition_1)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_1)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_1))]
                param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(
                    GoalPosition_2)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_2)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_2))]
                param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(
                    GoalPosition_3)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_3)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_3))]

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

                # if dxl_comm_result != COMM_SUCCESS:
                #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                # elif dxl_error != 0:
                #     print("%s" % packetHandler.getRxPacketError(dxl_error))

                while 1:
                    # Read present position
                    dxl_present_position_6, dxl_comm_result_6, dxl_error_6 = servo.read_pos(servo.DXL_ID_6)
                    dxl_present_position_1, dxl_comm_result_1, dxl_error_1 = servo.read_pos(servo.DXL_ID_1)
                    dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = servo.read_pos(servo.DXL_ID_2)
                    dxl_present_position_3, dxl_comm_result_3, dxl_error_3 = servo.read_pos(servo.DXL_ID_3)

                    dxl_present_position_6_deg = dxl_present_position_6 / 1023 * 300
                    dxl_present_position_1_deg = dxl_present_position_1 / 1023 * 300
                    dxl_present_position_2_deg = dxl_present_position_2 / 1023 * 300
                    dxl_present_position_3_deg = dxl_present_position_3 / 1023 * 300

                    # Print Statues
                    servo.print_status(servo.DXL_ID_6, GoalPosition_6_deg, dxl_present_position_6_deg)
                    servo.print_status(servo.DXL_ID_1, GoalPosition_1_deg, dxl_present_position_1_deg)
                    servo.print_status(servo.DXL_ID_2, GoalPosition_2_deg, dxl_present_position_2_deg)
                    servo.print_status(servo.DXL_ID_3, GoalPosition_3_deg, dxl_present_position_3_deg)

                    if not (abs(GoalPosition_6 - dxl_present_position_6) or abs(GoalPosition_1 - dxl_present_position_1) or
                            abs(GoalPosition_2 - dxl_present_position_2) or abs(
                                GoalPosition_3 - dxl_present_position_3)) > servo.DXL_MOVING_STATUS_THRESHOLD:
                        break
                if i == x-2:
                    exit(0)

    # Disable Dynamixel Torque
    servo.disable_servo_torque(servo.DXL_ID_6)
    servo.disable_servo_torque(servo.DXL_ID_1)
    servo.disable_servo_torque(servo.DXL_ID_2)
    servo.disable_servo_torque(servo.DXL_ID_3)

    # Close port
    servo.portHandler.closePort()
