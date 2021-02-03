import cv2

empty = cv2.imread("empty.png")
b1 = cv2.imread("image_ball_1.png")

i = b1 - empty

ret,thresh1 = cv2.threshold(i,1,255,cv2.THRESH_BINARY)

cv2.imshow("img", thresh1)

cv2.waitKey()