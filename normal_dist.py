import genetic
import time
import numpy as np
import matplotlib.pyplot as plt
import cma
cma.CMAOptions()
import pandas as pd
from scipy.signal import find_peaks


multipliers = 2
zero = 3

def fun(x):
    return x*2+1


fun1 = cma.ScaleCoordinates(fun, multipliers, zero)

def fun2(x):
    return fun(multipliers * (x - zero))

# fun2(x) == fun(multipliers * (x - zero))

print(fun1(1))
print(fun2(1))


import cma

cma.CMAOptions()
...
bounds='[None, None]  # lower (=bounds[0]) and upper domain boundaries, each a scalar or a list/vector'
...

x0 = [1000, 5, 1000, 5, 10000, 100]
stds = [200, 2, 200, 2, 1000, 20]
lb = [300, 0, 300, 0, 500, 0]
ub = [1500, 10, 1500, 10, 20000, 20]
es = cma.CMAEvolutionStrategy(x0=x0, sigma0=1.0, inopts={'CMA_stds': stds, 'bounds': [lb, ub]})
es.optimize(objective_function)