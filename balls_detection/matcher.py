import numpy as np
from scipy.spatial import distance_matrix


nc = np.array([[180, 494], [1070, 335], [466, 682]])
c = np.array([[1070, 335], [467, 682], [750, 120], [185, 494]])

distance = distance_matrix(c.reshape((-1,2)), nc)
print("distance\n", distance)


#matched =  np.empty((nc.shape[0],1,2), dtype=float)

matched = []
for k in range(c.shape[0]):
    v = np.min(distance[k,:])
    if v<50:
        ind = np.argmin(distance[k,:])
        matched.append(nc[ind])
        distance[k,ind] = 10000
    else :
        matched.append([10000,10000])

# if (len(nc)<len(c)):
#     for l in range(nc.shape[0]):
#         if (np.max(distance[:,l])!=10000):
#         #if (np.min(distance[:,l])>6):
#             matched.append(nc[l])

print("c\n", c)
print("nc\n", nc)
print("matched\n", np.asarray(matched).reshape((c.shape[0],1,2)))
#print("distance\n", distance)
