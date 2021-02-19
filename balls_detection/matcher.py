import numpy as np
from scipy.spatial import distance_matrix


nc = np.array([[466, 682], [551, 550],[550, 546], [180, 494], [1068, 334]])
c = np.array([[551, 547], [1070, 335], [467, 682], [185, 494]])

distance = distance_matrix(c.reshape((-1,2)), nc)
print("distance\n", distance)


#matched =  np.empty((nc.shape[0],1,2), dtype=float)

matched = []
for k in range(c.shape[0]):
    ind = np.argmin(distance[k,:])
    matched.append(nc[ind])
    distance[k,ind] = 10000

if (len(c)<len(nc)):
    for l in range(nc.shape[0]):
        if (np.max(distance[:,l])!=10000):
        #if (np.min(distance[:,l])>6):
            matched.append(nc[l])

print("c\n", c)
print("nc\n", nc)
print("matched\n", np.asarray(matched).reshape((nc.shape[0],1,2)))