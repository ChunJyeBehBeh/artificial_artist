import pdb
import time

import cv2
import numpy as np

import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

from skip import skip

class Drawer(object):
    """Instance of this class find the best path for inverse kinematics
    
    # Constructor Arguments:
        filename: input image absolute filepath.
        h_draw: height of end-effector while drawing.
        h_move: height of end-effector while moving.
    
    # Methods:
        find_closest
        findPathUtils
        findPath
        draw
    """
    @staticmethod
    def find_closest(pts, point):
        """This static method find the closest pixel from the current pixel to continue
        drawing when the current pixel does not have any neighbouring pixel with value 0
        
        # Arguments:
            pts: all remaining pixels' coordinates.
            point: the current pixel coordinates.
        """
        pts_arr = np.array(list(pts))
        dist_2 = np.sum((pts_arr - point) ** 2, axis=1)
        return pts_arr[np.argmin(dist_2)]

    def __init__ (self, filename, h_draw, h_move,display):
        self.h_draw = h_draw
        self.h_move = h_move
        self.display = display
        # Read image from path in grayscale
        self.arr = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        (h, w) = self.arr.shape[:2]
       
        (cX, cY) = (w / 2, h / 2)

        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise), then grab the sine and cosine
        # (i.e., the rotation components of the matrix)
        M = cv2.getRotationMatrix2D((cX, cY), 90, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])
        
        # compute the new bounding dimensions of the image
        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))

        # adjust the rotation matrix to take into account translation
        M[0, 2] += (nW / 2) - cX
        M[1, 2] += (nH / 2) - cY
        
        # perform the actual rotation and return the image
        self.arr = cv2.warpAffine(self.arr, M, (nW, nH))

        # Convert gray pixels into black
        self.arr[self.arr < 150] = 0
        
        # Find all black pixels
        self.pts = set(map(tuple, np.argwhere(self.arr == 0)))

    def findPathUtils(self, y, x):
        """Recursively (Depth-first search approach) visit remaining pixels.
        # Arguments:
            y: row index (height) of pixel
            x: column index (width) of pixel
        # Return:
            None
        """
        if self.point is None:
            self.path.append((y, x, self.h_move))
        else:
            self.pts.remove((y, x))
            if self.path[-1][2] == self.h_move:
                self.path.append((y, x, self.h_move))
        self.path.append((y, x, self.h_draw))
        self.point = (y, x)
        self.arr[y][x] = 255 # Mark pixel as visited
        
        flag = False

        # Check out all 8 neighbouring pixels
        for i in range(-1, 2):
            for j in range(-1, 2):
                try: # Handle IndexError exceptions (when pixel is at edge of image)
                    if self.arr[y + i][x + j] == 0 and (y + i, x + j) in self.pts: # If pixel is not visited
                        self.findPathUtils(y + i, x + j)
                        flag = True
                except Exception as e:
                    # print(e)
                    pass
        if not flag:
            self.path.append((y, x, self.h_move))

    def findPath(self):
        """Find the path (to-be-drawn pixels coordinates in specific order) that optimize
        inverse kinematics computation.
        # Arguments:
            None
        # Return:
            None
        """
        self.point = None
        self.path = []
        while (self.pts): # While there are still points we have not visited
            # print(len(self.pts))
            if self.point is None: # If the first point is visited
                y, x = self.pts.pop()
            else:
                # When we visit a pixel with non-zero neighbours, we find the
                # closest remaining pixel to continue drawing.
                y, x = self.find_closest(self.pts, self.point)

            self.findPathUtils(y, x)

    def draw(self):
        if(self.display):
            cv2.namedWindow("test", cv2.WINDOW_NORMAL)
            a = 0
            for y, x, h in self.path:
                a +=1
                print(a)
                print("y, x, h = {}, {}, {}".format(y, x, h))
                self.arr[y, x] = 0
                cv2.imshow("test", self.arr)
                key = cv2.waitKey(10) # Pause for 3 seconds before fetching next image
                if key == 27: # If ESC is pressed, exit loop
                    cv2.destroyAllWindows()
                    break
            cv2.imwrite("output.jpg",self.arr)
        return self.path 

def main():
    H_move = 6
    H_draw = -1.7
    filename = "Image/untitled.png"
    # filename = "Image/abc.jpeg"
    drawer = Drawer(filename, H_draw,H_move,False)
    drawer.findPath()
    arr = drawer.draw()
    arr=np.asarray(arr)         # <type 'numpy.ndarray'>

    # Factor x coordinate & y coordinate
    for i in arr:
        i[0] = i[0]*0.05
        i[1] = i[1]*0.05
    arr = np.round(arr,1)

    print("Number of point to IK: {}".format(len(arr)))

    arr = arr.tolist()
    
    arr = skip(arr,drawer.h_move,drawer.h_draw)
    arr=np.asarray(arr)
    print("After F Number of point to IK: {}".format(len(arr)))

    y=[]
    x=[]
    z=[]

    two_D_plot = True

    for i in arr:
        y.append(i[0])
        x.append(-i[1])         # flip the output in horizontal axis
        z.append(i[2])

    max_y = max(y)
    min_y = min(y)
    print("Max: {} Min: {} Offset: {}.".format(max_y,min_y,(max_y+min_y)/2))
    if two_D_plot:
        plt.title('Output Points forom Drawing')
        plt.scatter([i-(max_y+min_y)/2 for i in y], x)
        # plt.scatter(y, x)
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(arr[:,1],arr[:,0],arr[:,2])
    plt.show()

if __name__ == "__main__":
    main()
