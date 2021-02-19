import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo


from rclpy.qos import qos_profile_sensor_data
import tf2_py

import tf2_ros
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion
import cv2
import cv_bridge
from scipy.spatial import distance_matrix


class b_localizer(Node):
    def __init__(self):
        super().__init__("b_localizer")
        self.balls_publisher = self.create_publisher(Float32MultiArray, "/balls_coords", 10)
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback2, qos_profile=self.profile)
        

        self.visualize = False        
        self.bridge = cv_bridge.CvBridge()
        self.old_image = None
        self.old_coords = None
        self.old_gray = None
        self.backup = None
        self.first_spin = True
        self.lk_params = dict( winSize  = (15,15),
                            maxLevel = 10,
                            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.im_width = 1280
        self.im_height = 720
    
    def imgToWorld(self, px, py):
        u = px - self.im_width/2
        v = py - self.im_height/2
        x = -v
        y = -u
        wx = x*0.025
        wy = y*0.025
        return np.array([wx, wy])

    def buildMask(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # filtre
        # use the range_detector.py to find the values of lower and upper thresholds
        lower = np.array([20, 90, 90], dtype="uint8")
        upper = np.array([30, 255, 255], dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)

        # some operations on the mask to remove small blobs 
        mask = cv2.dilate(mask, None, iterations=3)
        mask = cv2.erode(mask, None, iterations=1)
        return mask

    def detectBalls(self, img):
        mask = self.buildMask(img)
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
            #print("Nombre de balles trouvés : ", detected_circles.shape[1])
            for pt in detected_circles[0,:]:
                x,y,r = pt[0], pt[1], pt[2]
                c = [x,y]
                ballsCoords.append(c)
                # draw the outer circle
                cv2.circle(img, (x,y), r, (0,255,0), 2)
                # draw the center of the circle
                cv2.circle(img, (x,y), 2, (0,0,255), 3)
        else:
            #print("No circle found")
            pass
        return np.float32(np.asarray(ballsCoords))

    def detectBalls2(self, img):
        mask = self.buildMask(img)
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
            #print("Nombre de balles trouvés : ", len(ballsCoords))
            #self.get_logger().info(f"Nombre de balles trouvés : {len(ballsCoords)}\n")
            #self.get_logger().info(f"Ball  spawned")
        else:
            #print("No circle found")
            self.get_logger().info("No circle found\n")
            pass
        return np.float32(np.asarray(ballsCoords))

    def im_callback2(self, msg):
        if self.first_spin:
            self.old_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # old coords
            self.old_gray = self.buildMask(self.old_image)
            self.old_coords = self.detectBalls2(self.old_image)
            self.first_spin = False
            return
        else:
            # read new image
            new_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
 
            # calculate optical flow    
            frame_gray = self.buildMask(new_frame)   
            coords1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.old_coords, None, **self.lk_params)
            good_new = coords1[(st==1).flatten()]
            good_old = self.old_coords[(st==1).flatten()]
            self.old_gray = frame_gray.copy()
            self.old_coords = good_new.reshape(-1,1,2)
            
            # Check for new balls
            newcoords = self.detectBalls2(new_frame)
            if (newcoords.shape[0]>self.old_coords.shape[0]):
                distance = distance_matrix(self.old_coords.reshape((-1,2)), newcoords)
                matched = []
                for k in range(self.old_coords.shape[0]):
                    ind = np.argmin(distance[k,:])
                    matched.append(newcoords[ind])
                    distance[k,ind] = 100000
                if len(self.old_coords)<len(newcoords):
                    for l in range(newcoords.shape[0]):
                        if (np.max(distance[:,l])!=100000):
                        #if (np.min(distance[:,l])>50):
                            matched.append(newcoords[l])
                self.old_coords = np.asarray(matched).reshape((newcoords.shape[0],1,2))
            
            coords = Float32MultiArray()
            lst = []
            if self.visualize:
            # visualization
                frame_show = new_frame.copy()
                for j in range(len(self.old_coords)):
                    a,b = self.old_coords[j][0][0], self.old_coords[j][0][1]
                    WX = self.imgToWorld(a, b)
                    lst.append(WX[0])
                    lst.append(WX[1])
                    #print("Ball : ", j, " ---->", WX)
                    frame_show = cv2.circle(frame_show, (a,b), 5, (0,200,0), -1)
                    frame_show = cv2.putText(frame_show, str(j), (int(a)+20,int(b)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                coords.data = lst
                self.balls_publisher.publish(coords)
                cv2.imshow("tracking", frame_show)
                cv2.waitKey(1)

            else:
                for j in range(len(self.old_coords)):
                    WX = self.imgToWorld(self.old_coords[j][0][0], self.old_coords[j][0][1])
                    lst.append(WX[0])
                    lst.append(WX[1])
                coords.data = lst
                self.balls_publisher.publish(coords)
                #self.get_logger().info(f"Balles : {coords.data}\n")


def main(args=None):
    rclpy.init(args=args)
    node = b_localizer()
    #print("go")
    cv2.namedWindow("tracking")
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()