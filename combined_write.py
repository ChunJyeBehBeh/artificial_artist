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
from skip import skip
from play_sound import *

# Parameters for Program Drawing
'''
Beh: check the best H_draw for the workspace
'''
# Pen length = 4.5cm
H_move = 8.0                   # variable + offset ->> 2+4
H_draw = 2.1                   # variable + offset ->> -2.3+4
filename = "Image/prof_low.png"

drawer = Drawer(filename,H_draw,H_move,False)
drawer.findPath()

# Program Parameters for Dynamixel Servos
print_param = True          # printing status of Dynamixel Servos
testing = False
min_X = 9
max_X = 35
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
        self.DEVICENAME = "COM3"  
        # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 10  # Dynamixel moving status threshold
        self.dxl_addparam_result = 0

        # for Protocol 1.0 Packet
        self.PKT_HEADER0 = 0
        self.PKT_HEADER1 = 1
        self.PKT_ID = 2
        self.PKT_LENGTH = 3
        self.PKT_INSTRUCTION = 4
        self.PKT_ERROR = 4
        self.PKT_PARAMETER0 = 5

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
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return dxl_present_position, dxl_comm_result, dxl_error

    # Function to read present angle for motor #id, output radians
    # Fix some error in library for read4ByteTxRx
    # def read4ByteTxRx(self, port, dxl_id, address):
    #     data, result, error = self.readTxRx(port, dxl_id, address, 4)
    #     data_read = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]),
    #                                 DXL_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
    #     return data_read, result, error

    # def readTxRx(self, port, dxl_id, address, length):
    #     PKT_PARAMETER0 = 5
    #     txpacket = [0] * 8
    #     data = []

    #     if dxl_id >= BROADCAST_ID:
    #         return data, COMM_NOT_AVAILABLE, 0

    #     txpacket[self.PKT_ID] = dxl_id
    #     txpacket[self.PKT_LENGTH] = 4
    #     txpacket[self.PKT_INSTRUCTION] = INST_READ
    #     txpacket[self.PKT_PARAMETER0 + 0] = address
    #     txpacket[self.PKT_PARAMETER0 + 1] = length

    #     rxpacket, result, error = self.txRxPacket(port, txpacket)
    #     if result == COMM_SUCCESS:
    #         error = rxpacket[self.PKT_ERROR]

    #         data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

    #     return data, result, error

    # def txRxPacket(self, port, txpacket):
    #     rxpacket = None
    #     error = 0

    #     # tx packet
    #     result = self.packetHandler.txPacket(port, txpacket)
    #     if result != COMM_SUCCESS:
    #         return rxpacket, result, error

    #     # (Instruction == BulkRead) == this function is not available.
    #     if txpacket[self.PKT_INSTRUCTION] == INST_BULK_READ:
    #         result = COMM_NOT_AVAILABLE

    #     # (ID == Broadcast ID) == no need to wait for status packet or not available
    #     if (txpacket[self.PKT_ID] == BROADCAST_ID):
    #         port.is_using = False
    #         return rxpacket, result, error

    #     # set packet timeout
    #     if txpacket[self.PKT_INSTRUCTION] == INST_READ:
    #         port.setPacketTimeout(txpacket[self.PKT_PARAMETER0 + 1] + 6)
    #     else:
    #         port.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

    #     # rx packet
    #     while True:
    #         rxpacket, result = self.rxPacket(port)
    #         if result != COMM_SUCCESS or txpacket[self.PKT_ID] == rxpacket[self.PKT_ID]:
    #             break

    #     if result == COMM_SUCCESS and txpacket[self.PKT_ID] == rxpacket[self.PKT_ID]:
    #         error = rxpacket[self.PKT_ERROR]

    #     return rxpacket, result, error

    # def rxPacket(self, port):
    #     rxpacket = []

    #     result = COMM_TX_FAIL
    #     checksum = 0
    #     rx_length = 0
    #     wait_length = 10  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)
    #     # FIX: CHANGED 6 TO 10 for read_pos

    #     while True:
    #         rxpacket.extend(port.readPort(wait_length - rx_length))
    #         rx_length = len(rxpacket)
    #         if rx_length >= wait_length:
    #             # find packet header
    #             for idx in range(0, (rx_length - 1)):
    #                 if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
    #                     break

    #             if idx == 0:  # found at the beginning of the packet
    #                 if (rxpacket[self.PKT_ID] > 0xFD) or (rxpacket[self.PKT_LENGTH] > RXPACKET_MAX_LEN) or (
    #                         rxpacket[self.PKT_ERROR] > 0x7F):
    #                     # unavailable ID or unavailable Length or unavailable Error
    #                     # remove the first byte in the packet
    #                     del rxpacket[0]
    #                     rx_length -= 1
    #                     continue

    #                 # re-calculate the exact length of the rx packet
    #                 if wait_length != (rxpacket[self.PKT_LENGTH] + self.PKT_LENGTH + 1):
    #                     wait_length = rxpacket[self.PKT_LENGTH] + self.PKT_LENGTH + 1
    #                     continue

    #                 if rx_length < wait_length:
    #                     # check timeout
    #                     if port.isPacketTimeout():
    #                         if rx_length == 0:
    #                             result = COMM_RX_TIMEOUT
    #                         else:
    #                             result = COMM_RX_CORRUPT
    #                         break
    #                     else:
    #                         continue

    #                 # calculate checksum
    #                 for i in range(2, wait_length - 1):  # except header, checksum
    #                     checksum += rxpacket[i]
    #                 checksum = ~checksum & 0xFF

    #                 # verify checksum
    #                 if rxpacket[wait_length - 1] == checksum:
    #                     result = COMM_SUCCESS
    #                 else:
    #                     result = COMM_RX_CORRUPT
    #                 break

    #             else:
    #                 # remove unnecessary packets
    #                 del rxpacket[0: idx]
    #                 rx_length -= idx

    #         else:
    #             # check timeout
    #             if port.isPacketTimeout():
    #                 if rx_length == 0:
    #                     result = COMM_RX_TIMEOUT
    #                 else:
    #                     result = COMM_RX_CORRUPT
    #                 break

    #     port.is_using = False

    #     #print "[RxPacket] %r" % rxpacket

    #     return rxpacket, result
    # # end fix

    # def read_pos(self, id):
    #     dxl_present_position, dxl_comm_result, dxl_error = self.read4ByteTxRx(self.portHandler, id, self.ADDR_MX_PRESENT_POSITION)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         return self.read_pos(id)
    #     elif dxl_error != 0:
    #         return self.read_pos(id)
    #     else:
    #         if(dxl_present_position > 1023 or dxl_present_position<0):
    #             return self.read_pos(id)
    #         return dxl_present_position, dxl_comm_result, dxl_error

    # Set servo move speed
    def set_joint_speed(self, id, speed):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_AX12A_MOVE_SPEED,
                                                                       int(speed))
        # print("Set Speed ",id,'---',dxl_comm_result,'---',dxl_error)
        if dxl_comm_result != COMM_SUCCESS:
            return self.set_joint_speed(id, speed)
        elif dxl_error != 0:
            return self.set_joint_speed(id, speed)
        else:
            print("Dynamixel#%d speed has been set: %s" % (id, speed))
            return 1

    def enable_servo_torque(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_MX_TORQUE_ENABLE,
                                                                       self.TORQUE_ENABLE)
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

    # Function to setup parameters for dynamixel sync write
    def add_params(self, id,params):
        dxl_addparam_result = self.groupSyncWrite.addParam(id, params)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed. Retrying..." % id)
            self.add_params(id,params)

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
        print("one by one")
        servo.goal_send(servo.DXL_ID_6, GoalPosition_6)
        servo.goal_send(servo.DXL_ID_3, GoalPosition_3)
        servo.goal_send(servo.DXL_ID_1, GoalPosition_1)
        servo.goal_send(servo.DXL_ID_2, GoalPosition_2)
    else:
        # Send goal to syncwrite storage
        servo.add_params(servo.DXL_ID_6, param_goal_position_6)
        servo.add_params(servo.DXL_ID_1, param_goal_position_1)
        servo.add_params(servo.DXL_ID_2, param_goal_position_2)
        servo.add_params(servo.DXL_ID_3, param_goal_position_3)

        # Syncwrite goal position
        dxl_comm_result = servo.groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("dynamixel_write result error %s" % servo.packetHandler.getTxRxResult(dxl_comm_result))
        
        # Clear syncwrite parameter storage
        servo.groupSyncWrite.clearParam()

    wait = time.time()
    print("Time: {}".format(wait))

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

        status_6 = (abs(GoalPosition_6 - dxl_present_position_6) <= servo.DXL_MOVING_STATUS_THRESHOLD)
        print(" Servo 6 Status: {}".format(status_6))
        print(GoalPosition_6)
        print(dxl_present_position_6)
        
        if ((abs(GoalPosition_6 - dxl_present_position_6) <= servo.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(GoalPosition_1 - dxl_present_position_1) <= servo.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(GoalPosition_2 - dxl_present_position_2) <= servo.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(GoalPosition_3 - dxl_present_position_3) <= servo.DXL_MOVING_STATUS_THRESHOLD)):
                break
        elif time.time() - wait > 10:
            print("arm is Blocked, breaking loop")
            break

def set_variable_speed():
    # Read Pos of Servo
    dxl6_present_position, _, _ = servo.read_pos(servo.DXL_ID_6)
    dxl1_present_position, _, _ = servo.read_pos(servo.DXL_ID_1)
    dxl2_present_position, _, _ = servo.read_pos(servo.DXL_ID_2)
    dxl3_present_position, _, _ = servo.read_pos(servo.DXL_ID_3)
    
    diff3 = abs(GoalPosition_3-dxl3_present_position)
    diff2 = abs(GoalPosition_2-dxl2_present_position)
    diff1 = abs(GoalPosition_1-dxl1_present_position)
    diff6 = abs(GoalPosition_6-dxl6_present_position)

    reduct = 1
    print("Angle difference is %s and reduction value is %s" %([diff1,diff2,diff3,diff6],reduct))
    if diff1 > 100:
        print("apply speed limit 1")
        diff2 = diff2 * reduct * 100/diff1
        diff3 = diff3 * reduct * 100/diff1
        diff6 = diff6 * reduct * 100/diff1
        diff1 = diff1 * reduct * 100/diff1
    if diff2 > 100:
        print("apply speed limit 2")
        diff1 = diff1 * reduct * 100/diff2
        diff3 = diff3 * reduct * 100/diff2
        diff6 = diff6 * reduct * 100/diff2
        diff2 = diff2 * reduct * 100/diff2
    if diff3 > 100:
        print("apply speed limit 3")
        diff1 = diff1 * reduct * 100/diff3
        diff2 = diff2 * reduct * 100/diff3
        diff6 = diff6 * reduct * 100/diff3
        diff3 = diff3 * reduct * 100/diff3
    if diff6 > 100:
        print("apply speed limit 4")
        diff1 = diff1 * reduct * 100/diff6
        diff2 = diff2 * reduct * 100/diff6
        diff3 = diff3 * reduct * 100/diff6
        diff6 = diff6 * reduct * 100/diff6

    diff1 = int(diff1)
    diff2 = int(diff2)
    diff3 = int(diff3)
    diff6 = int(diff6)

    if (diff1 == 0):
        diff1 = 1
    if (diff2 == 0):
        diff2 = 1
    if (diff3 == 0):
        diff3 = 1
    if (diff6 == 0):
        diff6 = 1

    servo.set_joint_speed(servo.DXL_ID_1,(diff1))
    servo.set_joint_speed(servo.DXL_ID_2,(diff2))
    servo.set_joint_speed(servo.DXL_ID_3,(diff3))
    servo.set_joint_speed(servo.DXL_ID_6,(diff6))


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

    # Set Speed of Servo
    joint_speed = 40
    servo.set_joint_speed(servo.DXL_ID_6, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_3, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_2, joint_speed)
    servo.set_joint_speed(servo.DXL_ID_1, joint_speed)

    # Go to home position 
    GoalPosition_3_deg, GoalPosition_3=program_input(150)
    GoalPosition_1 = GoalPosition_6 = GoalPosition_2 = GoalPosition_3
    GoalPosition_1_deg = GoalPosition_6_deg = GoalPosition_2_deg = GoalPosition_3_deg
    move_to(True)

    while 1:
        if servo.user_input:
            # User input goal position
            GoalPosition_3_deg, GoalPosition_3 = user_input(3)
            GoalPosition_6_deg, GoalPosition_6 = user_input(6)
            GoalPosition_2_deg, GoalPosition_2 = user_input(2)
            GoalPosition_1_deg, GoalPosition_1 = user_input(1)

            set_variable_speed()

            move_to(True)       # True: Not syncwrite

        else:            
            if not testing:
                print("Start drawing")
                # Drawing from input image
                arr = drawer.draw()
                print("Number of point to IK: {}".format(len(arr)))
                arr=np.asarray(arr)

                # Factor x coordinate & y coordinate
                for i in arr:
                    i[0] = i[0]*0.05
                    i[1] = i[1]*0.05

                arr = np.round(arr,1)                

                arr = arr.tolist() 
                arr = skip(arr,drawer.h_move,drawer.h_draw)
                print("After F Number of point to IK: {}".format(len(arr)))
                arr=np.asarray(arr)
                # from i =250, change here!!!
                arr=arr[250:]
                offset_y , offset_x = plot(arr,False,False)

            else:
                print("Testing")
                arr = [[min_X,max_Y,H_move],
                [min_X,max_Y,H_draw],
                [min_X,min_Y,H_draw],
                [max_X,min_Y,H_draw],
                [max_X,max_Y,H_draw],
                [min_X,max_Y,H_draw],
                [min_X,max_Y,H_move]]

            size_arr = len(arr)
            arr_store = arr  

            for index, i in enumerate(arr):
                '''
                Beh: Update this offset
                '''
                if(int(i[2])==int(H_move) or int(arr_store[index-1][2])==int(H_move)):
                    status_move = True
                else:
                    status_move = False
                    
                print("Status_move :{}".format(status_move))

                if not testing:
                    # Drawing from input image
                    print("Start sending the point")
                    x_coor = int(i[0])+ offset_x
                    y_coor = int(i[1])-offset_y
                    print("From Drawing for point {}/{}:".format(index+1,size_arr),i[0]+10-4, i[1]-15,i[2])
                    arr = ik.get_inverse(i[1]+offset_x, i[0]-offset_y,i[2])         

                else:
                    x_coor = int(i[0])
                    y_coor = int(i[1])

                    print("From Drawing: ",i[0], i[1],i[2])
                    val = sqrt(i[0]**2 + i[1]**2)
                    offset = get_offset(val)
                    print("val: {0} offset:{1}".format(val,offset))
                    
                    arr = ik.get_inverse(i[0], i[1],i[2])       

                arr[3] = -arr[3]
                arr = [i + 150.0 for i in arr]
                print("From IK(after +150): ",arr)
                
                GoalPosition_3_deg, GoalPosition_3 = program_input(arr[0])
                GoalPosition_6_deg, GoalPosition_6 = program_input(arr[1])
                GoalPosition_2_deg, GoalPosition_2 = program_input(arr[2])
                GoalPosition_1_deg, GoalPosition_1 = program_input(arr[3])

                set_variable_speed()

                if not testing:
                    if ((x_coor>min_X or x_coor<max_X) and (y_coor>min_Y or y_coor<max_Y)):
                        move_to(True)
                    else:
                        warnings.warn('Exceed the Limit. Skip that point')
                else:
                    move_to(True)

                print("--- Wait ---")
                '''
                Beh: check the min duration between two points (power suppy)
                '''
                time.sleep(1.0)
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
