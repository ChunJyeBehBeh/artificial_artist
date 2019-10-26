def skip(arr,height_move):
    arr.reverse() #reverse the array

    arr1 = [] #new array

    
    a = 0 #initialise
    for i in reversed(arr): #reverse again
        y = i[0]
        x = i[1]
        h = i[2]

        if h == height_move: #when height is h_move
            arr1.append(i)
            arr.remove(i)
            a = 1
            
        elif h == 0.1 and a == 1: #first h_draw point after h_move, dont need to remove
            arr1.append((y, x, h))
            arr.remove(i)
            a = 2
            
        elif h == 0.1 and (a == 2 or a == 3): #remove 2 points continuosly,if height is h_draw
            arr.remove(i)
            if a == 3:
                a =1
            else:
                a = a+1
    return arr1

if __name__ == "__main__":
    test=[[0,0,9],[0,0,0],[1,1,0],[1,0,0],[2,2,0],[3,3,0],[3,3,9]]
    print(type(test))
    output=skip(test,9)
    print(output)
