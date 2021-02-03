"""
import cv2

empty = cv2.imread("empty.png")
b1 = cv2.imread("image_ball_1.png")

i = b1 - empty

ret, thresh = cv2.threshold(i, 1, 255, cv2.THRESH_BINARY)

cv2.namedWindow("img")
cv2.imshow("img", thresh)
cv2.waitKey(0)
"""
import  cv2
import numpy as np

def nothing(x):
    pass

# Create a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('dp','image',1,10,nothing)
cv2.createTrackbar('md','image',1,100,nothing)
cv2.createTrackbar('p1','image',1,255,nothing)
cv2.createTrackbar('p2','image',1,255,nothing)
cv2.createTrackbar('mr','image',1,255,nothing)
cv2.createTrackbar('mx','image',1,255,nothing)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('dp', 'image', 1)
cv2.setTrackbarPos('md', 'image', 10)
cv2.setTrackbarPos('p1', 'image', 100)
cv2.setTrackbarPos('p2', 'image', 5)
cv2.setTrackbarPos('mr', 'image', 5)
cv2.setTrackbarPos('mx', 'image', 11)

# Initialize to check if HSV min/max value changes
dp = md = p1 = p2 = mr = mx = 0


img = cv2.imread("./balls/image_ball_72.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.medianBlur(gray, 5)
# bgr --> hsv
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# filtre
#blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
# use the rage_detector.py to find the values of lower and upper thresholds
lower = np.array([20, 90, 90], dtype="uint8")
upper = np.array([60, 255, 255], dtype="uint8")
mask = cv2.inRange(hsv, lower, upper)
# some operations on the mask to remove small blobs 
mask = cv2.dilate(mask, None, iterations=3)
mask = cv2.erode(mask, None, iterations=1)

save = img.copy()
output = mask
waitTime = 33

while(1):
    img = save.copy()
    output = mask
    # get current positions of all trackbars
    dp = cv2.getTrackbarPos('dp','image')
    md = cv2.getTrackbarPos('md','image')
    p1 = cv2.getTrackbarPos('p1','image')
    p2 = cv2.getTrackbarPos('p2','image')
    mr = cv2.getTrackbarPos('mr','image')
    mx = cv2.getTrackbarPos('mx','image')


    # Hough Cicrle
    detected_circles = cv2.HoughCircles(mask,
        method=cv2.HOUGH_GRADIENT, # detection method 
        dp=dp,  # inverse ratio of resolution
        minDist=md, # Minimum distance between detected centers.
        param1=p1, # Upper threshold for the internal Canny edge detector
        param2=p2, # Threshold for center detection
        minRadius=mr,
        maxRadius=mx
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

    # Display output image
    cv2.imshow('image', img)

    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(waitTime) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()