# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
from numpy import radians, degrees, cos, sin, arctan, sqrt, array
from scipy.optimize import minimize

length = [2, 14.5, 13.3, 5.4]


def x_end(t1, theta):
    ''' Function to return x-value of position vector
    -------------------------------------------------
    theta: List[3]= theta2, theta3, theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''

    x = length_2 * sin(theta2) * (sin(theta0) - cos(theta0) * sin(theta1)) + length_1 * cos(theta0) * cos(
        theta1) + length_3 * cos(theta3) * (
                    sin(theta2) * (sin(theta0) - cos(theta0) * sin(theta1)) + cos(theta0) * cos(theta1) * cos(
                theta2)) + length_3 * sin(theta3) * (
                    cos(theta2) * (sin(theta0) - cos(theta0) * sin(theta1)) - cos(theta0) * cos(theta1) * sin(
                theta2)) + length_2 * cos(theta0) * cos(theta1) * cos(theta2)
    return x


def y_end(t1, theta):
    ''' Function to return x-value of position vector
    -------------------------------------------------
    theta: List[3]= theta2, theta3, theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    y = length_1 * cos(theta1) * sin(theta0) - length_2 * sin(theta2) * (
            cos(theta0) + sin(theta0) * sin(theta1)) - length_3 * cos(theta3) * (
                sin(theta2) * (cos(theta0) + sin(theta0) * sin(theta1)) - cos(theta1) * cos(theta2) * sin(
            theta0)) - length_3 * sin(theta3) * (
                cos(theta2) * (cos(theta0) + sin(theta0) * sin(theta1)) + cos(theta1) * sin(theta0) * sin(
            theta2)) + length_2 * cos(theta1) * cos(theta2) * sin(theta0)
    return y


def z_end(t1, theta):
    '''Function to return z_value of position vector
    ------------------------------------------------
    theta: List[3]= theta2, theta3, theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    z = length_0 + length_1 * sin(theta1) + length_3 * cos(theta3) * (
            cos(theta1) * sin(theta2) + cos(theta2) * sin(theta1)) + length_3 * sin(theta3) * (
                cos(theta1) * cos(theta2) - sin(theta1) * sin(theta2)) + length_2 * cos(theta1) * sin(
        theta2) + length_2 * cos(theta2) * sin(theta1)
    return z


def end_constraint(theta):
    '''Constraint of sum (theta2 to theta4) must be 180 to keep pen parallel to paper
    ---------------------------------------------------------------------------------
    theta: List[3] of theta2 to theta 4'''
    return sum(theta) - 180


def objective(theta, t1, x, y, z):
    ''' Custom objective of RMS Error to calculate distance. 
    Find error between result and target value'''
    dist = sqrt((x_end(t1, theta) - x) ** 2) + sqrt((y_end(t1, theta) - y) ** 2) + sqrt((z_end(t1, theta) - z) ** 2)
    print("Current epoch RMS Error: {}".format(dist))
    return dist


def get_inverse(x, y, z):
    ''' Funtion to get inverse kinematics
    -----------------INPUT---------------
    x,y,z = target end coordinates
    ----------------RETURN---------------
    Returns list of theta1,2,3,4'''

    b = (-180, 180)  # Arbitrary bounds for now
    bnds = (b, b, b)  # Bounds for theta2, theta3, theta4
    cons = [
        {'type': 'eq', 'fun': end_constraint}]  # Define constraint type and function to feed into scipy  constraints

    def _get_theta(x, y):
        '''Function to get theta1
        '''
        try:
            t1 = degrees(arctan(y / x))
            return t1
        except ZeroDivisionError:  # Catch Error for when y=0 raise ZeroDivisionError
            return 0

    t1 = _get_theta(x, y)
    x0 = array([0, 0, 0])

    sol = minimize(objective, x0, args=(t1, x, y, z), method='SLSQP', bounds=bnds, constraints=cons,
                   options={'disp': True})
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
    return [x, y, z]


# Script for testing. Compare p1 and p1_prime
p1 = [0.4, 0.2, 0.3]
p1_sol = get_inverse(p1[0], p1[1], p1[2])
p1_prime = get_forward(p1_sol)
