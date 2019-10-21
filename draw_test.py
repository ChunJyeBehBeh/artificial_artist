import pdb
import time

import cv2
import numpy as np

class Drawer(object):
    def __init__ (self, filename):
        self.arr = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        self.arr[self.arr < 150] = 0
        self.pts = set(map(tuple, np.argwhere(self.arr == 0)))

    def find_closest(self):
        pts_arr = np.array(list(self.pts))
        dist_2 = np.sum((pts_arr - self.point)**2, axis=1)
        return pts_arr[np.argmin(dist_2)]

    def findPathUtils(self, y, x):
        self.point = (y, x)
        self.arr[y][x] = 255
        self.path.append((y, x))

        for i in range(-1, 2):
            for j in range(-1, 2):
                try:
                    if self.arr[y + i][x + j] == 255 and (y + j, x + i) in self.pts:
                        self.pts.remove((y + j, x + i))
                        self.findPathUtils(y + j, x + i)
                except Exception as e:
                    print(e)
                    pass

    def findPath(self):
        self.point = None
        self.path = []
        while (self.pts):
            if self.point is None:
                y, x = self.pts.pop()
            else:
                y, x = self.find_closest()

            self.findPathUtils(y, x)

    def draw(self):
        '''cv2.namedWindow("test", cv2.WINDOW_NORMAL)
        for y, x in self.path:
            self.arr[y, x] = 0
            cv2.imshow("test", self.arr)
            key = cv2.waitKey(10)#pauses for 3 seconds before fetching next image
            if key == 27:#if ESC is pressed, exit loop
                cv2.destroyAllWindows()
                break'''
        return self.path
       

def main():
    filename = "Love.png"
    drawer = Drawer(filename)
    drawer.findPath()
    drawer.draw()

if __name__ == "__main__":
    main()
