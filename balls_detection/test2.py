import  cv2
import numpy as np



img = cv2.imread("./video/image_ball_88.png")
# bgr --> hsv
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# filtre
# use the rage_detector.py to find the values of lower and upper thresholds
lower = np.array([20, 90, 90], dtype="uint8")
upper = np.array([60, 255, 255], dtype="uint8")
mask = cv2.inRange(hsv, lower, upper)

# some operations on the mask to remove small blobs 
mask = cv2.dilate(mask, None, iterations=3)
mask = cv2.erode(mask, None, iterations=1)

# Hough Cicrle
detected_circles = cv2.HoughCircles(mask,
    method=cv2.HOUGH_GRADIENT, # detection method 
    dp=1,  # inverse ratio of resolution
    minDist=10, # Minimum distance between detected centers.
    param1=100, # Upper threshold for the internal Canny edge detector
    param2=5, # Threshold for center detection
    minRadius=5,
    maxRadius=11
    )

if detected_circles is not None :
    detected_circles = np.uint16(np.around(detected_circles))
    print("Nombre de balles trouvés : ", detected_circles.shape[1])
    for pt in detected_circles[0,:]:
        x,y,r = pt[0], pt[1], pt[2]
        # draw the outer circle
        cv2.circle(img, (x,y), r, (0,255,0), 2)
        # draw the center of the circle
        cv2.circle(img, (x,y), 2, (0,0,255), 3)
else:
    print("No circle found")

cv2.namedWindow('image')
cv2.imshow('image', mask)
cv2.namedWindow('image2')
cv2.imshow('image2', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
