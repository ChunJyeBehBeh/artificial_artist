# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
from numpy import degrees, cos, sin, arctan, sqrt, array
from numpy import radians as r
from scipy.optimize import minimize, basinhopping

length = [2, 14.5, 13.3, 5.4]


def x_end(theta):
    ''' Function to return x-value of position vector
    -------------------------------------------------
    theta: List[4]= theta[0], theta[1], theta[2], theta[3]  List of theta angles
    length: List[4] = length0, length1, length2, length3, length3 List of joint length'''

    x = length[1]*cos(r(theta[0]))*sin(r(theta[1])) - length[3]*sin(r(theta[3]))*(cos(r(theta[0]))*cos(r(theta[1]))*sin(r(theta[2])) + cos(r(theta[0]))*cos(r(theta[2]))*sin(r(theta[1]))) - length[3]*cos(r(theta[3]))*(cos(r(theta[0]))*sin(r(theta[1]))*sin(r(theta[2])) - cos(r(theta[0]))*cos(r(theta[1]))*cos(r(theta[2]))) + length[2]*cos(r(theta[0]))*cos(r(theta[1]))*cos(r(theta[2])) - length[2]*cos(r(theta[0]))*sin(r(theta[1]))*sin(r(theta[2]))

    return x


def y_end(theta):
    ''' Function to return x-value of position vector
    -------------------------------------------------
    theta: List[4]= theta[0], theta[1], theta[2], theta[3]  List of theta angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    y = length[1]*sin(r(theta[0]))*sin(r(theta[1])) - length[3]*sin(r(theta[3]))*(cos(r(theta[1]))*sin(r(theta[0]))*sin(r(theta[2])) + cos(r(theta[2]))*sin(r(theta[0]))*sin(r(theta[1]))) - length[3]*cos(r(theta[3]))*(sin(r(theta[0]))*sin(r(theta[1]))*sin(r(theta[2])) - cos(r(theta[1]))*cos(r(theta[2]))*sin(r(theta[0]))) + length[2]*cos(r(theta[1]))*cos(r(theta[2]))*sin(r(theta[0])) - length[2]*sin(r(theta[0]))*sin(r(theta[1]))*sin(r(theta[2]))
    return y


def z_end(theta):
    '''Function to return z_value of position vector
    ------------------------------------------------
    theta: List[3]= r(theta[2]), r(theta[3]), theta4  List of theta angles
    alpha: List[4]= alpha1, alpha2, alpha3, alpha4  List of alpha angles
    length: List[4] = length1, length2, length3, length4 List of joint length'''
    z = length[0] + length[1]*cos(r(theta[1])) - length[3]*cos(r(theta[3]))*(cos(r(theta[1]))*sin(r(theta[2])) + cos(r(theta[2]))*sin(r(theta[1]))) - length[3]*sin(r(theta[3]))*(cos(r(theta[1]))*cos(r(theta[2])) - sin(r(theta[1]))*sin(r(theta[2]))) - length[2]*cos(r(theta[1]))*sin(r(theta[2])) - length[2]*cos(r(theta[2]))*sin(r(theta[1]))
    return z


def end_constraint(theta):
    '''Constraint of sum (r(theta[2]) to theta4) must be 180 to keep pen parallel to paper
    ---------------------------------------------------------------------------------
    theta: List[3] of r(theta[2]) to theta 4'''
    return sum(theta) - 180


def objective(theta, x, y, z):
    ''' Custom objective of RMS Error to calculate distance. 
    Find error between result and target value'''
    dist = sqrt((x_end(theta) - x) ** 2) + sqrt((y_end(theta) - y) ** 2) + sqrt((z_end(theta) - z) ** 2)
    print("Current epoch RMS Error: {}".format(dist))
    return dist


def get_inverse(x, y, z):
    ''' Funtion to get inverse kinematics
    -----------------INPUT---------------
    x,y,z = target end coordinates
    ----------------RETURN---------------
    Returns list of r(theta[1]),2,3,4'''

    b = (-150, 150)  # Arbitrary bounds for now
    bnds = (b, b, b, b)  # Bounds for r(theta[2]), r(theta[3]), theta4

    # cons = [
    # {'type': 'eq', 'fun': end_constraint}]  # Define constraint type and function to feed into scipy  constraints

    def _get_theta(x, y):
        '''Function to get r(theta[1])
        '''
        try:
            t1 = degrees(arctan(y / x))
            return t1
        except ZeroDivisionError:  # Catch Error for when y=0 raise ZeroDivisionError
            return 0

    # t1 = _get_theta(x, y)
    x0 = array([0.3, 0.3, 0.3, 0.3])
    minimizer_kwargs = {"method": "SLSQP", "args": (x, y, z), "bounds": bnds}
    # sol = minimize(objective, x0, args=(x, y, z), method='SLSQP', bounds=bnds,
    # options={'disp': True})
    sol = basinhopping(objective, x0, niter=40, minimizer_kwargs=minimizer_kwargs, disp=True)
    angles = sol.x

    return angles


def get_forward(theta):
    '''Forward Kinematics function to test for get_inverse
    ----------INPUT--------
    theta: List of angles from r(theta[1]),2,3,4 (Designed to take in output of get_inverse())
    ---------OUTPUT--------
    return list of x,y,z coordinates'''
    x = x_end(theta)
    y = y_end(theta)
    z = z_end(theta)
    return [x, y, z]


if __name__ == "__main__":
    # Script for testing. Compare p1 and p1_prime
    #p1 = [20, 20, 2]
    p1 = [18.7, 0, 16.5]
    p1_sol = np.round(get_inverse(p1[0], p1[1], p1[2]), 3)
    p1_prime = get_forward(p1_sol)
