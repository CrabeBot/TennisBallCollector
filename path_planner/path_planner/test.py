import numpy as np

A = np.array(((0, 0), (1, 0), (1, 1), (0, 1)))
B = np.array(((1, 1)))
C = A + B
print(A, '\n\n', B, '\n\n', C)

DX, DY = 30, 35

WIDHT = 1280 - 2 * DX
HEIGHT = 720 - 2 * DY

print(WIDHT, HEIGHT)