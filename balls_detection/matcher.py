import numpy as np
from scipy.spatial import distance_matrix


nc = np.array([[466, 682], [550, 546], [180, 494], [1068, 334]])
c = np.array([[551, 547], [1070, 335], [467, 682]])

distance = distance_matrix(c.reshape((-1,2)), nc)
print("distance\n", distance)


#matched =  np.empty((nc.shape[0],1,2), dtype=float)

matched = []
for k in range(c.shape[0]):
    #matched[k,0] = nc[np.argmin(distance[k,:])]
    matched.append(nc[np.argmin(distance[k,:])])

for k in range(nc.shape[0]):
    if (np.min(distance[:,k])>100):
        matched.append(nc[k])

print("c\n", c)
print("nc\n", nc)
print("matched\n", np.asarray(matched).reshape((nc.shape[0],1,2)))