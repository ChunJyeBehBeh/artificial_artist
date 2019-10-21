# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
from numpy import radians,degrees, cos, sin, arctan, sqrt, array
from scipy.optimize import minimize
alpha = [15.80]
length = [0,143.07,132.92,49.94]
def x_end(t1, theta):
    ''' Function to return x-value of position vector
    -------------------------------------------------
    theta: List[3]= theta2, theta3, theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    
    x = (length[1] * sin(radians(theta[0] + alpha[0])) + \
        length[2] * cos(radians(theta[1] +theta[0])) + \
        length[3] * cos(radians(sum(theta)))) * cos(radians(t1))
    return x

def y_end(t1, theta):
    ''' Function to return x-value of position vector
    -------------------------------------------------
    theta: List[3]= theta2, theta3, theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    y = (length[1] * sin(radians(theta[0] + alpha[0])) + \
        length[2] * cos(radians(theta[0] + theta[1])) + \
        length[3] * cos(radians(sum(theta)))) * sin(radians(t1))
    return y

def z_end(t1, theta):
    '''Function to return z_value of position vector
    ------------------------------------------------
    theta: List[3]= theta2, theta3, theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    z = length[1] * cos(radians(theta[0] + alpha[0])) + length[2] * sin(radians(theta[1] + theta[0])) + length[3] * sin(sum(theta))
    return z    

def end_constraint(theta):
    '''Constraint of sum (theta2 to theta4) must be 180 to keep pen parallel to paper
    ---------------------------------------------------------------------------------
    theta: List[3] of theta2 to theta 4'''
    return sum(theta) - 180

def objective(theta, t1, x, y ,z):
    ''' Custom objective of RMS Error to calculate distance. 
    Find error between result and target value'''
    dist = sqrt((x_end(t1, theta) - x)**2) + sqrt((y_end(t1, theta) - y)**2) + sqrt((z_end(t1, theta) - z)**2)
    print("Current epoch RMS Error: {}".format(dist))
    return dist

def get_inverse(x, y, z):
    ''' Funtion to get inverse kinematics
    -----------------INPUT---------------
    x,y,z = target end coordinates
    ----------------RETURN---------------
    Returns list of theta1,2,3,4'''
    
    b = (-150, 150)     # Arbitrary bounds for now
    bnds = (b, b, b )   # Bounds for theta2, theta3, theta4
    cons = [{'type': 'eq', 'fun': end_constraint}] # Define constraint type and function to feed into scipy  constraints
    def _get_theta(x,y):
        '''Function to get theta1
        '''
        try:
            t1 = degrees(arctan(y/x))
            return t1
        except ZeroDivisionError: #Catch Error for when y=0 raise ZeroDivisionError
            return 0 
    t1 = _get_theta(x, y)
    x0 = array([0,0,0])
    
    sol = minimize(objective,x0, args=(t1,x, y, z), method='SLSQP', bounds=bnds, options={'disp':True} )
    print("TEST")
    print(sol.x)
    angles = np.insert(sol.x, 0, t1)
    
    return angles

def get_forward(theta):
    '''Forward Kinematics function to test for get_inverse
    ----------INPUT--------
    theta: List of angles from theta1,2,3,4 (Designed to take in output of get_inverse())
    ---------OUTPUT--------
    return list of x,y,z coordinates''' 
    t1 = theta[0]
    theta = theta[1:]
    x = x_end(t1, theta)
    y = y_end(t1, theta)
    z = z_end(t1, theta)
    return [x,y,z]

#Script for testing. Compare p1 and p1_prime  
p1 = [0.4,0.2,0.3]
p1_sol = get_inverse(p1[0], p1[1], p1[2])
p1_prime = get_forward(p1_sol)


