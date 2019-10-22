from PID import PID
from dynamixel_sdk import *

# Control table address
ADDR_MX_TORQUE_ENABLE = 24  # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_SPEED = 38

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_6 = 6
BAUDRATE = 1000000  # Dynamixel default baudrate : 57600
DEVICE_PORT = "/dev/ttyUSB0"  # Check which port is being used on your controller

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
MAX_SPEED = 20


def setup():
    # Create handler objects
    portHandler = PortHandler(DEVICE_PORT)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port and set baudrate
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)

    # Enable Dynamixel
    _, _ = packetHandler.write1ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

    # Limit speed
    _, _ = packetHandler.write2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_MOVING_SPEED, 20)

    speed, _, _ = packetHandler.read2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_MOVING_SPEED)
    print("Speed of servo ID {} is {}.".format(DXL_ID_6, speed))

    # Set to vertical 
    _, _ = packetHandler.write2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_GOAL_POSITION, 512)

    return portHandler, packetHandler


def pid():
    # PID Parameters
    KP = 0
    KI = 0.1
    KD = 0
    SAMPLE_TIME = 0.01
    UPPER_LIMIT = 200
    LOWER_LIMIT = -100

    # Create PID 
    pid = PID()

    pid.sample_time = SAMPLE_TIME
    pid.tunings = (KP, KI, KD)
    pid.output_limit = (LOWER_LIMIT, UPPER_LIMIT)
    pid.setpoint = 512
    pid.auto_mode = False

    return pid


if __name__ == "__main__":
    portHandler, packetHandler = setup()
    pid = pid()

    # Function to read present position
    def read_pos(id):
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

    currentpos, dxl_comm_result, dxl_error = read_pos(DXL_ID_6)

    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
    print("Current position from read2ByteTxRx is {}.".format(currentpos))
    print("Target from PID setpoint is {}.".format(pid.setpoint))
    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")

    # Enable PID to ensure vertical
    pid.auto_mode = True

    # Get current speed 
    speed, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_PRESENT_SPEED)

    if speed < 2:
        while True:
            # Get current position
            currentpos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID_6,
                                                                                 ADDR_MX_PRESENT_POSITION)
            # Compute a new output value
            output = int(pid(currentpos)) + 512
            print("Current Pos: {} Output from PID: {}.".format(currentpos,output))

            # Output PID to motor
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID_6, ADDR_MX_GOAL_POSITION,
                                                                      output)

            if abs(currentpos - 512) < 1:
                break
    
    print("Closing Port")
    # currentpos, dxl_comm_result, dxl_error = read_pos(DXL_ID_6)

    # print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
    # print("Current position from read2ByteTxRx is {}.".format(currentpos))
    # print("Target from PID setpoint is {}.".format(pid.setpoint))
    # print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
    portHandler.closePort()
