import numpy as np
import os

for root, dirs, files in os.walk(os.getcwd(), topdown=False):
    for name in files:
        if root.count(os.sep) != 6: continue        
        if name.endswith("log"):
            print(name)
            x = np.loadtxt(name , usecols = (0))
            print(name, np.mean(x))
