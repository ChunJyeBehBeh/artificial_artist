from dynamixel_sdk import *
import time
import ctypes



def syncwrite_storage(id, goal_position):
        # Add Dynamixel#n goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(id, goal_position)
        if dxl_addparam_result != 1:
            print(dxl_addparam_result)
            print("[ID:%03d] groupSyncWrite addparam failed" % (id))
            quit()

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_1                      = 1                 
DXL_ID_2                      = 2
DXL_ID_3                      = 3                 
DXL_ID_6                      = 6
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
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
time.sleep(3.0)             # wait for 3 seconds

speed = 50
# Limit speed
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_MOVING_SPEED, speed)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_MOVING_SPEED, speed)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_MOVING_SPEED, speed)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_MOVING_SPEED, speed)

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_MOVING_SPEED)
print("Speed of ID 1 is {}.".format(speed))
print("The value should be 100.")
print("-----------------------------")

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_MOVING_SPEED)
print("Speed of ID 2 is {}.".format(speed))
print("The value should be 200.")
print("-----------------------------")

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_MOVING_SPEED)
print("Speed of ID 3 is {}.".format(speed))
print("The value should be 300.")
print("-----------------------------")

speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_MOVING_SPEED)
print("Speed of ID 6 is {}.".format(speed))
print("The value should be 400.")
print("-----------------------------")

GoalPosition_6=512
GoalPosition_1=512
GoalPosition_2=512
GoalPosition_3=512

param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_6)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_6)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_6)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_6))]
param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_1)), DXL_HIBYTE(DXL_LOWORD(
    GoalPosition_1)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_1)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_1))]
param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_2)), DXL_HIBYTE(DXL_LOWORD(
    GoalPosition_2)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_2)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_2))]
param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(GoalPosition_3)), DXL_HIBYTE(DXL_LOWORD(
        GoalPosition_3)), DXL_LOBYTE(DXL_HIWORD(GoalPosition_3)), DXL_HIBYTE(DXL_HIWORD(GoalPosition_3))]

syncwrite_storage(DXL_ID_6, param_goal_position_6)
syncwrite_storage(DXL_ID_1, param_goal_position_1)
syncwrite_storage(DXL_ID_2, param_goal_position_2)
syncwrite_storage(DXL_ID_3, param_goal_position_3)

# Syncwrite goal position
# dxl_comm_result = groupSyncWrite.txPacket()

# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

# # Clear syncwrite parameter storage
# groupSyncWrite.clearParam()

# Set to vertical 
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_3, ADDR_MX_GOAL_POSITION, 512)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_GOAL_POSITION, 512)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_2, ADDR_MX_GOAL_POSITION, 512)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_1, ADDR_MX_GOAL_POSITION, 512)
