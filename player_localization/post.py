import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp2d
from scipy.optimize import minimize
df = pd.read_csv("measures.csv")

dv = df.values

detected = dv[:, :2]
true = dv[:, 2:]

correction = true - detected

correctionNorm = np.linalg.norm(correction, axis = 1)
dstCtr = np.linalg.norm(detected, axis = 1)
plt.scatter(dstCtr, correctionNorm)

p = np.polyfit(dstCtr, correctionNorm, 1)

a = np.array([0, 15])
plt.plot(a, p[0] * a + p[1])

print(p[0])

plt.show()