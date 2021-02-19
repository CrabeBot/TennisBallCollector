import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp2d
from scipy.optimize import minimize
df = pd.read_csv("measure.csv")

dv = df.values
norm = np.linalg.norm(dv[:, :2], axis=1)
mask = norm < 20
print(mask[:10])
d = dv[mask]

print(d.shape, norm[mask].shape)

x_interp = interp2d(d[:, 2], d[:, 3], norm[mask])


plt.quiver(d[:, 2], d[:, 3], d[:, 0], d[:, 1], color='blue')
plt.show()

fig = plt.figure()
ax = fig.gca(projection='3d')

x = np.linspace(-15, 15, 100)
y = np.linspace(-15, 15, 100)

Z = np.empty((100, 100))
for idx, i in enumerate(x):
    for idy, j in enumerate(y):
        Z[idx, idy] = x_interp(j, i)


Y, X = np.meshgrid(y, x)

ax.scatter(d[:, 2], d[:, 3], norm[mask])

# surf = ax.plot_surface(X, Y, Z,
#                     linewidth=0, antialiased=False)

xa = np.linspace(0, 20, 100)
def fun(x):
    points = np.linalg.norm(d[:, 2:] - x, axis=1)
    print(points.shape)
    p = np.polyfit(points, norm[mask], 1)
    interp = points*p[0] + p[1]

    return np.sum((interp - points.T)**2)

plt.show()
plt.figure()


plt.scatter(np.linalg.norm(d[:, 2:] - np.array([-3, -1.5]), axis=1), norm[mask])
p = np.polyfit(np.linalg.norm(d[:, 2:] - np.array([-3, -1.5]), axis=1), norm[mask], 1)
interp = xa*p[0] + p[1]

plt.scatter(xa, interp)
plt.show()

print("Polyfit output : {}".format(p))