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

show_animation = True
dt = 0.0001

if show_animation:
    plt.ion()


def show_image(img):
    plt.imshow(img, cmap='gray')
    plt.show()


def plot_diagram(x, y):
    if show_animation:
        plt.plot(x, y, 'g*')
        # plt.show()
        plt.pause(dt)


if __name__ == '__main__':
    fig = plt.figure()
    image = cv2.imread('test.png', 0)  # read image data
    height, width = image.shape[:2]  # Get height and width
    # Scale image to between 0 and 1
    plt.imshow(image, cmap='gray')
    for j in range(0, height):
        for i in range(0, width):
            # 200 is an arbitrary value
            if image[j][i] < 200:
                image[j][i] = 0
            else:
                image[j][i] = 1

    # Reduce width of drawing line to width of around 1 through dilation
    kernel = np.ones((3, 3), np.uint8)
    dilation = cv2.dilate(image, kernel, iterations=1)
    # cv2.imwrite('result.jpg', dilation)

    print(dilation)
    counter =0
    for j in range(1, height - 1):
        for i in range(1, width - 1):
            if (int(dilation[j - 1][i - 1] + dilation[j][i - 1] + dilation[j + 1][i - 1] + dilation[j - 1][i] +
                    dilation[j + 1][i] + dilation[j - 1][i + 1] + dilation[j][i + 1] + dilation[j + 1][i + 1]) ==1):
                counter=counter+1
                print(j,i)
                plot_diagram(i, j)
    print(counter)
    plt.savefig('test_1.png')
