import cv2
import numpy as np
from scipy.spatial import distance_matrix

def buildMask(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # filtre
    # use the rage_detector.py to find the values of lower and upper thresholds
    lower = np.array([20, 90, 90], dtype="uint8")
    upper = np.array([60, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower, upper)

    # some operations on the mask to remove small blobs 
    mask = cv2.dilate(mask, None, iterations=3)
    mask = cv2.erode(mask, None, iterations=1)
    return mask

def detectBalls(img):
    mask = buildMask(img)
    # Hough Cicrle
    detected_circles = cv2.HoughCircles(mask,
                                        method=cv2.HOUGH_GRADIENT, # detection method 
                                        dp=1,  # inverse ratio of resolution
                                        minDist=10, # Minimum distance between detected centers.
                                        param1=100, # Upper threshold for the internal Canny edge detector
                                        param2=5, # Threshold for center detection
                                        minRadius=5,
                                        maxRadius=30
                                        )
    ballsCoords = []
    if detected_circles is not None :
        detected_circles = np.uint16(np.around(detected_circles))
        print("Nombre de balles trouvés : ", detected_circles.shape[1])
        for pt in detected_circles[0,:]:
            x,y,r = pt[0], pt[1], pt[2]
            c = [x,y]
            ballsCoords.append(c)
            # draw the outer circle
            cv2.circle(img, (x,y), r, (0,255,0), 2)
            # draw the center of the circle
            cv2.circle(img, (x,y), 2, (0,0,255), 3)
    else:
        print("No circle found")
    
    return np.float32(np.asarray(ballsCoords))

def detectBalls2(img):
    mask = buildMask(img)
    cnts, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    ballsCoords = []
    if len(cnts) > 0:
        for cnt in cnts:
            ((x, y), r) = cv2.minEnclosingCircle(cnt)
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) 
            cy = int(M["m01"] / M["m00"])
            c = [cx,cy]
            ballsCoords.append(c)
            # draw the outer circle
            cv2.circle(img, (cx,cy), int(r), (0,255,0), 2)
            # draw the center of the circle
            cv2.circle(img, (cx,cy), 2, (0,0,255), 3)
        print("Nombre de balles trouvés : ", len(ballsCoords))
    else:
        print("No circle found")
    return np.float32(np.asarray(ballsCoords))

#-------------------------------------------------------------------------------------
old_frame = cv2.imread("./video/image_ball_0.png")
old_gray = buildMask(old_frame)
coords0 = detectBalls2(old_frame)
#print("coords0 : ", coords0)

cv2.namedWindow("tracking")
#cv2.namedWindow("mask")
waitTime = 20

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 10,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))
# Create a mask image for drawing purposes
traj = np.zeros_like(old_frame)

i = 0
while(i<100):
    print(i)
    frame = cv2.imread("./video/image_ball_{0}.png".format(str(i)))
    
    # calculate optical flow    
    frame_gray = buildMask(frame)
    
    coords1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, coords0, None, **lk_params)

    #print("coords1 : ", coords1)
    #print("st : ", st)
    #print("err : ", err)
    # Select good points
    good_new = coords1[(st==1).flatten()]
    good_old = coords0[(st==1).flatten()]

    # draw the tracks
    for j,(new,old) in enumerate(zip(good_new, good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        traj = cv2.line(traj, (a,b), (c,d), (74,74,74), 2)
        frame = cv2.circle(frame, (a,b), 5, (0,200,0), -1)
        frame = cv2.putText(frame, str(j), (int(a)+20,int(b)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

    img = cv2.add(frame, traj)

    cv2.imshow("tracking", img)
    #cv2.imshow("mask", frame_gray)
    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(waitTime) & 0xFF == ord('q'):
        break

    old_gray = frame_gray.copy()
    coords0 = good_new.reshape(-1,1,2)
#new try    
    newcoords = detectBalls2(frame)
    # print("newcoords", newcoords.shape)
    # print("coords0", coords0.shape)
    print("=======================================")
    if (newcoords.shape[0]>coords0.shape[0]):
        distance = distance_matrix(coords0.reshape((-1,2)), newcoords)
        #matched =  np.empty((newcoords.shape[0],1,2))
        matched = []
        # print("distance\n", distance)
        # print("newcoords\n", newcoords)
        # print("coords0\n", coords0)
        # print("newcoords : ", newcoords.shape)
        # print("coords0 : ", coords0.shape)
        for k in range(coords0.shape[0]):
            #matched[k,0] = newcoords[np.argmin(distance[k,:])]
            matched.append(newcoords[np.argmin(distance[k,:])])
        for k in range(newcoords.shape[0]):
            if (np.min(distance[:,k])>100):
                matched.append(newcoords[k])
        coords0 = np.asarray(matched).reshape((newcoords.shape[0],1,2))
        #coords0 = matched
        #print("matched", coords0.shape)

    i +=1

cv2.waitKey(0)
cv2.destroyAllWindows()


# Works but don't keeps the balls sorted
    # Now update the previous frame and previous points
"""
    # check for new ball
    newcoords = detectBalls(frame)
    print("newcoords", newcoords.shape)
    print("coords0", coords0.shape)
    print("=======================================")
    if (newcoords.shape[0]>coords0.shape[0]):
        coords0 = newcoords.reshape((newcoords.shape[0],1,2))
"""
#=================================================================