'''Beh: There are some issues at the starting point
# Ideally is h_move->h_draw ..... h_draw -> h_move
# Another idea to further reduce the point
# If only one point between h_move, actually we can just remove that stupid lonely point
# h_move->h_draw->h_move  ===> h_move->h_move 
reject=[]
counter = 0
for index,i in enumerate(arr):
    if i[2]==drawer.h_move:
        print("test")
    else:
        # print(counter)
        counter+=1                      # Issue that I mentioned yesterday, the ending part issue
        if(counter!=30 and (arr[index+1])[2]!=drawer.h_move):
            reject.append(index)        # save the index so that can use the np.deleted() function
        else:
            # print("keep")
            counter=0                   # keep the tenth point and reset the counter
arr=np.asarray(arr)                     # convert list to array so that can use np.delete()
arr=np.delete(arr,reject,axis=0)        # (original array, index that need to be deleted, axis)
    '''