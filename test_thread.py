from threading import Thread
from draw import *

H_move = -1.5+4
H_draw = -2.2+4
filename = "Image/Love.png"
drawer = Drawer(filename,H_draw,H_move,True)

def func1():
    print 'Working'
    arr_1 = drawer.findPath()
    
def func2():
    print 'Working'
    arr_2 = drawer.findPath()

if __name__ == '__main__':
    Thread(target = func1).start()
    Thread(target = func2).start()