import numpy as np
from tqdm import tqdm

a = np.array([1,2,3],[])
_, idx = np.unique(a, return_index=True)
print(idx)
print(a[np.sort(idx)])

for index, value in enumerate(a): 
    print(index,":",value)