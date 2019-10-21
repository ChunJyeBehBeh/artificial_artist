from dynamixel_sdk import *
import time
import ctypes
import argparse

def syncwrite_storage(id, goal_position):
    # Add Dynamixel#n goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(id, goal_position)
    if dxl_addparam_result != 1:
        print(dxl_addparam_result)
        print("[ID:%03d] groupSyncWrite addparam failed" % (id))
        syncwrite_storage(id, goal_position)

# Function to read present position
def read_pos(id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    return dxl_present_position

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_4                      = 4                 
DXL_ID_5                      = 5                 
BAUDRATE                      = 1000000             # Dynamixel default baudrate : 57600
DEVICE_PORT                   = "/dev/ttyUSB0"          # Check which port is being used on your controller

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
MAX_SPEED                   = 20

# Create handler objects
portHandler = PortHandler(DEVICE_PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port and set baudrate
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler,
                                             ADDR_MX_GOAL_POSITION, 4)

# Enable Dynamixel
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_5, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

# time.sleep(3.0)             # wait for 3 seconds

speed_motor_4 = 50
speed_motor_5 = 60

# Limit speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_MOVING_SPEED, speed_motor_4)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_5, ADDR_MX_MOVING_SPEED, speed_motor_5)

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_MOVING_SPEED)
print("Speed of ID 4 is {}.".format(speed))
print("The value should be {}.".format(speed_motor_4))
print("-----------------------------")

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_5, ADDR_MX_MOVING_SPEED)
print("Speed of ID 5 is {}.".format(speed))
print("The value should be {}.".format(speed_motor_5))
print("-----------------------------")

def test():
    # Input goal position
    dxl4_goal_position = int(input("Motor 4 Goal pos?"))
    dxl5_goal_position = int(input("Motor 5 Goal pos?"))

    if(args.syncwrite):
        print("Syncwrite")
        # Allocate goal position value into byte array
        param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(dxl4_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl4_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl4_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl4_goal_position))]
        param_goal_position_5 = [DXL_LOBYTE(DXL_LOWORD(dxl5_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl5_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl5_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl5_goal_position))]

        syncwrite_storage(DXL_ID_4, param_goal_position_4)
        syncwrite_storage(DXL_ID_5, param_goal_position_5)

        # Limit speed
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_MOVING_SPEED, speed_motor_4)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_5, ADDR_MX_MOVING_SPEED, speed_motor_5)

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
        # # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        # print("Test")

    else:
        # Set to vertical 
        dxl_comm_result, _ = packetHandler.write2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_GOAL_POSITION, dxl4_goal_position)
        dxl_comm_result, _ = packetHandler.write2ByteTxRx(portHandler, DXL_ID_5, ADDR_MX_GOAL_POSITION, dxl5_goal_position)

    time.sleep(10.0)
    dxl4_present_position = read_pos(DXL_ID_4)
    dxl5_present_position = read_pos(DXL_ID_5)


    print("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d" % (DXL_ID_4, dxl4_present_position, DXL_ID_5, dxl5_present_position))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dynamixel Servo Control')
    parser.add_argument('--syncwrite', action='store_true', help='use this argument if want to syncwrite')
    args = parser.parse_args()

    while(1):
        test()