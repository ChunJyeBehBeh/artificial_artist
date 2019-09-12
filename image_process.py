# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 15:27:54 2019

@author: chuac
"""

import cv2
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from matplotlib import style

fig = plt.figure()
image = cv2.imread('test.png', 0) #read image data
height, width = image.shape[:2] #Get height and width
#Scale image to between 0 and 1
for j in range(0, height):
        for i in range(0, width):
            #200 is an arbitrary value
            if image[j][i] < 200:
                image[j][i] = 0
            else:
                image[j][i] = 1

#Reduce width of drawing line to width of around 1 through dilation
kernel = np.ones((3,3),np.uint8)                
dilation = cv2.dilate(image,kernel,iterations = 1)
dilation *=255
cv2.imwrite('result.jpg', dilation)