import numpy as np

def skip(arr,height_move,height_draw):
    arr1 = [] #new array

    
    a = 0 #initialise

    for i in range(0,len(arr)):
        y = arr[i][0]
        x = arr[i][1]
        h = arr[i][2]
        
        if h == height_move: #when height is h_move
            arr1.append(arr[i])
            a = 1
            
        elif h == height_draw and a == 1: #first h_draw point after h_move, dont need to remove
            arr1.append(arr[i])
            a = 2
       
        elif arr[i+1][2] == height_move:
            arr1.append(arr[i])
        
        elif h == height_draw and (a in range(2,8)): #remove 4 points continuosly,if height is h_draw
            if a == 7:
                a =1
            else:
                a = a+1
    return arr1

if __name__ == "__main__":
    test=[[0,0,9],[0,0,0],[1,1,0],[1,0,0],[2,2,0],[3,3,0],[3,3,9]]
    test=np.asarray(test)
    for index,i in enumerate(test):
        print("No. {}: {}".format(index,i))
        if (index==0):
            print(i)
        else:
            print(i[2]==9 or int(test[index-1][2])==9)
    # print(type(test))
    # output=skip(test,9,0)
    # print(output)
