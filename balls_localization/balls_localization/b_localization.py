import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo

from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.node import Node
import tf2_py

import tf2_ros
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion
import cv2
import cv_bridge
from scipy.spatial import distance_matrix

from tf2_msgs.msg import TFMessage

class Balle(object):
    def __init__(self, x, y, nm, det=1, is_v=1):
        self.coords = [x, y]
        self.detected = det
        self.is_visible = is_v
        self.catched = 0
        self.num = nm
        self.distance_to_robot = 0


class b_localizer(Node):
    def __init__(self):
        super().__init__("b_localizer")
        self.balls_publisher = self.create_publisher(Float32MultiArray, "/balls_coords", qos_profile=qos_profile_sensor_data)
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)
        
        self.visualize = False
        self.bridge = cv_bridge.CvBridge()
        
        self.first_spin = True
        self.old_image = None
        self.old_coords = None
        self.old_gray = None
        self.lk_params = dict( winSize  = (15,15),
                    maxLevel = 10,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.im_width = 1280
        self.im_height = 720

        self.balls = []
        self.nb_ball_spawn = 0


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

    def detectBalls(self, mask):
        cnts, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballsCoords = []
        if len(cnts) > 0:
            for cnt in cnts:
                M = cv2.moments(cnt)
                if (M["m00"]!=0):
                    cx = int(M["m10"] / M["m00"]) 
                    cy = int(M["m01"] / M["m00"])
                    c = [cx, cy]
                    ballsCoords.append(c)
        else:
            #print("No circle found")
            self.get_logger().info("No circle found\n")
            pass
        return np.float32(np.asarray(ballsCoords))

    def im_callback(self, msg):
        if self.first_spin:
            self.old_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # old coords
            self.old_gray = self.buildMask(self.old_image)
            self.old_coords = self.detectBalls(self.old_gray)
            for c in self.old_coords:
                x, y = c[0], c[1]
                self.balls.append(Balle(x, y, self.nb_ball_spawn, 1, 1))
                self.nb_ball_spawn += 1
            self.first_spin = False
        else:
            # read new image
            new_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            lk_params = dict( winSize  = (15,15),
                            maxLevel = 10,
                            criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        
            # calculate optical flow    
            frame_gray = self.buildMask(new_frame)
            
            coords1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.old_coords, None, **self.lk_params)

            good_new = coords1[(st==1).flatten()]
            good_old = self.old_coords[(st==1).flatten()]

            self.old_gray = frame_gray.copy()
            self.old_coords = good_new.reshape(-1,1,2)
            
            # Check for new balls
            newcoords = self.detectBalls(frame_gray)
            distance = distance_matrix(self.old_coords.reshape((-1,2)), newcoords)
            
            if (newcoords.shape[0]==self.old_coords.shape[0]):
                #print("same number")
                for k in range(self.old_coords.shape[0]):
                    v = np.min(distance[k,:])
                    ind = np.argmin(distance[k,:])
                    x, y = self.old_coords[k][0][0], self.old_coords[k][0][1]
                    self.balls[k].coords = [x, y]
                    self.balls[k].num = ind
                    self.balls[k].is_visible = 1
            # If any new : 
            elif (newcoords.shape[0]>self.old_coords.shape[0]):
                matched = []
                for k in range(self.old_coords.shape[0]):
                    ind = np.argmin(distance[k,:])
                    matched.append(newcoords[ind])
                    distance[k,ind] = 100000
                if len(self.old_coords)<len(newcoords):
                    for l in range(newcoords.shape[0]):
                        if (np.max(distance[:,l])!=100000):
                            matched.append(newcoords[l])
                self.old_coords = np.asarray(matched).reshape((newcoords.shape[0],1,2))
            
            # Create Balles Objects
            for j in range(len(self.old_coords)):
                x, y = self.old_coords[j][0][0], self.old_coords[j][0][1]
                if (len(self.balls)<len(self.old_coords)):
                    self.balls.append(Balle(x, y, self.nb_ball_spawn, 1, 1))
                self.balls[j].coords = [x, y]
                self.balls[j].num = j
                self.balls[j].is_visible = 1
                self.balls[j].detected = 1
                self.nb_ball_spawn += 1

            if (newcoords.shape[0]<self.old_coords.shape[0]):
                matched = []
                dis = []
                for k in range(self.old_coords.shape[0]):
                    v = np.min(distance[k,:])
                    ind = np.argmin(distance[k,:])
                    if v>30:
                        #matched.append(newcoords[ind])
                        #distance[k,ind] = 10000
                        dis.append(k)
                    else :
                        #matched.append(self.old_coords[0][k])
                        zebbi = 1                        
                #print("dis : ", dis)
                #print("============")
                #self.old_coords = np.asarray(matched).reshape((self.old_coords.shape[0],1,2))
                #print("self.old_coords : ", self.old_coords)
                for j in range(len(self.balls)):
                    #self.balls[j].coords = [self.old_coords[j][0][0], self.old_coords[j][0][1]]
                    if (j in dis):
                        for b in self.balls:
                            #print("ball ", b.num, " : ", b.coords[0], ", ", b.coords[1])
                            if (b.num==j):
                                x, y = self.old_coords[j][0][0], self.old_coords[j][0][1]
                                self.balls[j].coords = [x, y]
                                self.balls[j].is_visible = 0
            if self.visualize:
                frame_show = new_frame.copy()
                for b in self.balls:
                    x,y = b.coords[0], b.coords[1]
                    #print("x, y", x, ",", y)
                    j = b.num
                    if (b.is_visible):
                        frame_show = cv2.circle(frame_show, (int(x), int(y)), 5, (0,200,0), -1)
                        frame_show = cv2.putText(frame_show, str(j), (int(x)+20,int(y)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                    else:
                        frame_show = cv2.circle(frame_show, (int(x), int(y)), 5, (0,0,200), -1)
                        frame_show = cv2.putText(frame_show, str(j), (int(x)+20,int(y)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,200), 2)                

                cv2.imshow("tracking", frame_show)
                cv2.waitKey(1)
            lst = []
            for b in self.balls:
                if (b.detected and b.is_visible):
                    w = self.imgToWorld(b.coords[0], b.coords[1])
                    lst.append(float(b.num))
                    lst.append(w[0])
                    lst.append(w[1])
            
            lst_coords = Float32MultiArray()
            lst_coords.data = [1.0, 2.0, 3.0]
            #print("lst : ", lst)
            lst_coords.data = lst
            #print("lst_coords.data : ", len(lst_coords.data))
            self.balls_publisher.publish(lst_coords)


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