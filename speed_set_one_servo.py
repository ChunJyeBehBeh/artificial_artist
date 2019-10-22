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
    print("Reading pose")
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, id,
                                                                                        ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        # [TxRxResult] Incorrect status packet!
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        read_pos(id)
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        read_pos(id)

    return dxl_present_position, dxl_comm_result, dxl_error

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_4                      = 6                
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

# time.sleep(3.0)             # wait for 3 seconds

speed_motor_4 = 50

# Limit speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_MOVING_SPEED, speed_motor_4)

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_MOVING_SPEED)
print("Speed of ID 4 is {}.".format(speed))
print("The value should be {}.".format(speed_motor_4))
print("-----------------------------")

def test():
    # Input goal position
    dxl4_goal_position = int(input("Motor %d Goal pos?"%DXL_ID_4))

    if(args.syncwrite):
        print("Syncwrite")
        # Allocate goal position value into byte array
        param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(dxl4_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl4_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl4_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl4_goal_position))]

        syncwrite_storage(DXL_ID_4, param_goal_position_4)

        # Limit speed
        dxl_comm_result, _ = packetHandler.write2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_MOVING_SPEED, speed_motor_4)

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
        # # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

    else:
        # Set to vertical 
        dxl_comm_result, _ = packetHandler.write2ByteTxRx(portHandler, DXL_ID_4, ADDR_MX_GOAL_POSITION, dxl4_goal_position)

    time.sleep(2.0)
    dxl4_present_position,_,_ = read_pos(DXL_ID_4)


    print("[ID:%03d] PresPos:%03d" % (DXL_ID_4, dxl4_present_position))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dynamixel Servo Control')
    parser.add_argument('--syncwrite', action='store_true', help='use this argument if want to syncwrite')
    args = parser.parse_args()

    while(1):
        test()