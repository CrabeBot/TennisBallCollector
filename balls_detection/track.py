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
                                        maxRadius=20
                                        )
    ballsCoords = []
    if detected_circles is not None :
        detected_circles = np.uint16(np.around(detected_circles))
        print("Nombre de balles trouvés : ", detected_circles.shape[1])
        i = 0
        for pt in detected_circles[0,:]:
            x,y,r = pt[0], pt[1], pt[2]
            c = [x,y]
            ballsCoords.append(c)
            i += 1
            # draw the outer circle
            cv2.circle(img, (x,y), r, (0,255,0), 2)
            # draw the center of the circle
            cv2.circle(img, (x,y), 2, (0,0,255), 3)
    else:
        print("No circle found")
    
    return np.float32(np.asarray(ballsCoords))

#-------------------------------------------------------------------------------------
old_frame = cv2.imread("./video/image_ball_0.png")
old_gray = buildMask(old_frame)
coords0 = detectBalls(old_frame)
#print("coords0 : ", coords0)

cv2.namedWindow("tracking")
waitTime = 20

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))
# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

i = 0
while(i<100):
    #print(i)
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
        mask = cv2.line(mask, (a,b), (c,d), color[i].tolist(), 2)
        frame = cv2.circle(frame, (a,b), 5, color[i].tolist(), -1)
        frame = cv2.putText(frame, str(j), (int(a)+20,int(b)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

    img = cv2.add(frame, mask)

    cv2.imshow("tracking", img)
    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(waitTime) & 0xFF == ord('q'):
        break

    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    coords0 = good_new.reshape(-1,1,2)
    # check for new ball
    newcoords = detectBalls(frame)
    print("newcoords", newcoords.shape)
    print("coords0", coords0.shape)
    print("=======================================")
    if (newcoords.shape[0]>coords0.shape[0]):
        # for i in range(newcoords.shape[0]):
        #     x = newcoords[i,0]
        #     ind = np.where(coords0==x)


        coords0 = newcoords.reshape((newcoords.shape[0],1,2))
    i +=1

cv2.destroyAllWindows()


    # newcoords = detectBalls(frame)
    # print("newcoords", newcoords.shape)
    # print("coords0", coords0.shape)
    # print("=======================================")
    # if (newcoords.shape[0]>coords0.shape[0]):
    #     # for i in range(newcoords.shape[0]):
    #     #     x = newcoords[i,0]
    #     #     ind = np.where(coords0==x)
    #     distance = distance_matrix(coords0.reshape((-1,2)), newcoords)
    #     matched =  np.empty((newcoords.shape[0],1,2))
    #     print("distance", distance.shape)
    #     print("newcoords", newcoords.shape)
    #     for k in range(newcoords.shape[0]):
    #         matched[k,0] = newcoords[np.argmin(distance[k,:])]

    #     coords0 = matched
    #     print("matched", coords0.shape)