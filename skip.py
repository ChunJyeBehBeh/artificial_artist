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
        
        elif h == height_draw and (a in range(2,8)): #remove 3 points continuosly,if height is h_draw
            if a == 7:
                a =1
            else:
                a = a+1

    arr2 = []
    b = 0
    c = 0
    for j in range(0,len(arr1)):
        y = arr1[j][0]
        x = arr1[j][1]
        h = arr1[j][2]
        
        if h == height_move: #when height is h_move
            arr2.append(arr1[j])
            b = 1
        
        elif b == 1: #check the point after h = h_move
            if (arr1[j][2] == height_draw and arr1[j+1][2] == height_move): #if only one stroke is drawn after between two lift up,then skip both points
                b = 2
            else: #if more than one stroke, append into new array
                arr2.append(arr1[j])
                b = 0
            
        elif b == 2: #to skip point
            b = 0
                
        else: #append those normal points into new array
            arr2.append(arr1[j])
            
    return arr2

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
