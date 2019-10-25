import numpy as np
from tqdm import tqdm

arr=[[0,0,0],[0,0,9],[1,1,9],[1,1,0],[2,2,0],[3,3,0],[3,3,9]]
arr=np.asarray(arr)
arr = arr.tolist()

print(arr)

reject = []
for index,i in enumerate(arr):
    if(i[2]==9):
        # reject.append(index)
        print("9")
        break
    else:
        print("0")
    print("test")
# arr=np.asarray(arr)
# arr=np.delete(arr,reject,axis=0)
# print(arr)