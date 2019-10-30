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
import sys
# from dynamixel_sdk import *  # Uses Dynamixel SDK library
from skip import skip
# from play_sound import *

# "D:/Robotics/artificial_artist/Robotics3DSimulations"
sys.path.append("../artificial_artist/simulation")

import Segment as sg
import RobotArm as ra

# Parameters for Program Drawing
'''
Beh: check the best H_draw for the workspace
'''
# Pen length = 4.5cm
H_move = 8.0                   # variable + offset ->> 2+4
H_draw = 2.1                   # variable + offset ->> -2.3+4
filename = "Image/abc.jpeg"

drawer = Drawer(filename,H_draw,H_move,False)
drawer.findPath()

# Program Parameters for Dynamixel Servos
print_param = True          # printing status of Dynamixel Servos
testing = False
min_X = 10
max_X = 30
min_Y =-15
max_Y = 15

#Unit vectors in x,y,z direction for convenience 
ex = np.matrix([[1],[0],[0]])
ey = np.matrix([[0],[1],[0]])
ez = np.matrix([[0],[0],[1]])

#############################
#Create the segments that make up robot. Segment is initialized in the following form
# sg.Segment(type of joint, length, unit vector, zero configuration angle)
s1 = sg.Segment(0,0,ez,0)     # <-- Base servo
s2 = sg.Segment(2,1.00,ez,0)  # <- link
s3 = sg.Segment(0,0,ey,0)
s4 = sg.Segment(2,1.00,ez,0)
s5 = sg.Segment(2,2.00,ex,0)
s6 = sg.Segment(0,0,ey,0)
s7 = sg.Segment(2,1.00,ex,0)
s8 = sg.Segment(0,0,ey,0)
s9 = sg.Segment(2,1.00,ex,0)
s10 = sg.Segment(2,1.0,ez,0)

#Organize segments into a list, order matters!
segments = [s1,s2,s3,s4,s5,s6,s7,s8,s9]             # <- fix
#Organize zero config angles, order matters!
Q = [0,0,0,0,0,0,0,0,0]                             # <- fix

#construct robot arm from segment list and zero config list
r1 = ra.RobotArm(segments,Q)

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
    val = float(input("Enter Goal Position for Motor {} in degrees (Range is 0 to 300 degrees; 150 degrees is neutral position):".format(id)))
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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Dynamixel Servo Control')
    parser.add_argument('--user_input', action='store_false', help='ignore this argument in order to ask goal from program')
    args = parser.parse_args()

    while 1:
        if args.user_input:
            # User input goal position
            GoalPosition_3_deg, GoalPosition_3 = user_input(3)
            GoalPosition_6_deg, GoalPosition_6 = user_input(6)
            GoalPosition_2_deg, GoalPosition_2 = user_input(2)
            GoalPosition_1_deg, GoalPosition_1 = user_input(1)

            GoalPosition_3_rad = np.radians(GoalPosition_3_deg)
            GoalPosition_6_rad = np.radians(GoalPosition_6_deg)
            GoalPosition_2_rad = np.radians(GoalPosition_2_deg)
            GoalPosition_1_rad = np.radians(GoalPosition_1_deg)
            
            input_angle = [GoalPosition_3_rad,0,GoalPosition_6_rad,0,0,GoalPosition_2_rad,0,GoalPosition_1_rad,0]
            r1.zeroC = input_angle
            r1.update(segments)
            r1.drawArm()

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
                offset_y , offset_x = plot(arr,False,False)

            else:
                print("Testing")
                '''
                Beh: Draw a Square along the workspace (power suppy)
                '''
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
                arr = [i for i in arr]      # no +150 here
                print("From IK(after +150): ",arr)
                
                GoalPosition_3_deg, GoalPosition_3 = program_input(arr[0])
                GoalPosition_6_deg, GoalPosition_6 = program_input(arr[1])
                GoalPosition_2_deg, GoalPosition_2 = program_input(arr[2])
                GoalPosition_1_deg, GoalPosition_1 = program_input(arr[3])

                GoalPosition_3_rad = np.radians(GoalPosition_3_deg)
                GoalPosition_6_rad = np.radians(GoalPosition_6_deg)
                GoalPosition_2_rad = np.radians(GoalPosition_2_deg)
                GoalPosition_1_rad = np.radians(GoalPosition_1_deg)
                
                input_angle = [GoalPosition_3_rad,0,GoalPosition_6_rad,0,0,GoalPosition_2_rad,0,GoalPosition_1_rad,0]
                r1.zeroC = input_angle
                r1.update(segments)

                if not testing:
                    if ((x_coor>min_X or x_coor<max_X) and (y_coor>min_Y or y_coor<max_Y)):
                        r1.drawArm()
                    else:
                        warnings.warn('Exceed the Limit. Skip that point')
                else:
                    r1 = ra.RobotArm(segments,Q)

                print("--- Wait ---")
                '''
                Beh: check the min duration between two points (power suppy)
                '''
                time.sleep(1.0)
            break

    print("Finished")
    # play_sound()
    
