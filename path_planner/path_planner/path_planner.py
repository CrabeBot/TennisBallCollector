import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo

from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.node import Node
import tf2_py


import tf2_ros
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion, Pose2D
import cv2
import cv_bridge
from scipy.spatial.distance import cdist

from tf2_msgs.msg import TFMessage


from shapely.geometry import Polygon, LineString, MultiPolygon, Point

DX, DY = 30, 35

PX_WIDHT = 1280 - 2 * DX
PX_HEIGHT = 720 - 2 * DY
WIDHT = 30
HEIGHT = 16



DEBUG = True

def rect(l, L, x=0, y=0):
    return np.array(((x, y), (x+l, y), (x+l, y+L), (x, y+L)))

def rect_from_center(l, L, x=0, y=0):
    return np.array(((x-l/2, y-L/2), (x+l/2, y-L/2), (x+l/2, y+L/2), (x-l/2, y+L/2)))

def world_to_img(X):
    X = np.array(list(X)).reshape(len(X), -1)
    X = (-X[::-1, :]*PX_WIDHT/WIDHT + np.array(((DX + PX_WIDHT/2, DY + PX_HEIGHT/2)))).astype(int)
    return X

def img_to_world(x, y):
    pass

def cout(msg):
    rclpy.logging.get_logger("path_finder").info(msg)


class path_planner(Node):
    def __init__(self):
        super().__init__("wps")
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)
        self.obj_subscriber = self.create_subscription(Pose2D, "/objectif", self.obj_callback, qos_profile=self.profile)
        # self.obstacles_subscriber = self.create_subscription(Pose2D, "/objectif", self.obstacles_callback, qos_profile=self.profile)

        self.br = tf2_ros.TransformBroadcaster(self)

        self.l_rob = 0.5
        self.obstacles_fixes()
        self.terrain = Polygon(rect_from_center(WIDHT, HEIGHT)).buffer(-self.l_rob)

        self.pos = (-5, 0)
        self.path = LineString(((0, 0), (0, 0)))
        a = Polygon(((101, 100), (101, 101), (100, 101)))
        self.polys = MultiPolygon((a, a))
        
    def obstacles_fixes(self):

        l_box, L_box = 2.7, 2.3

        l_filet, L_filet = 0.3, 13

        self.obstacles = MultiPolygon([
            Polygon(rect(l_box, L_box, -WIDHT/2, -HEIGHT/2)), 
            Polygon(rect(l_box, L_box, WIDHT/2 - l_box, HEIGHT/2 - L_box)), 
            Polygon(rect_from_center(l_filet, L_filet))
            ]).buffer(self.l_rob)

    def rec_path(self, A, B, d, sens=0):
  
        # print("d : ", d)
        line = LineString((A, B))
        # if d>5:
        #     return (A, B), d
        if not self.terrain.contains(line):
            return None, 0

        for obs in self.obstacles:
            if line.crosses(obs):
                coords = (obs.union(line)).convex_hull.exterior.coords
                i = list(coords).index(A)
                if sens != 0:
                    #print("AHHHH ! : ", (i+sens)%len(list(coords)))
                    C = coords[(i+sens)%len(list(coords))]
                    # print("C1, C2 : ", C1, C2)
                    pts, d0 = self.rec_path(C, B, d, sens=sens)
                    if pts == None:
                        return None, 0
                    d += d0
                else:
                    C1, C2 = coords[i+1], coords[(i-2)%len(list(coords))]
                    
                    # print("C1, C2 : ", C1, C2)
                    pts1, d1 = self.rec_path(C1, B, d, sens=1)
                    pts2, d2 = self.rec_path(C2, B, d, sens=-2)
                    if d1 > d2 or pts1 == None:
                        C = C2
                        pts = pts2
                        d += d2
                        if pts == None:
                            return None, 0
                    else:
                        C = C1
                        pts = pts1
                        d += d1

                d += cdist(np.array(A).reshape(1, -1), np.array(C).reshape(1, -1))
                pts.insert(0, A)
                return pts, d


        d += cdist(np.array(A).reshape(1, -1), np.array(B).reshape(1, -1))
        return [A, B], d

    def im_callback(self, msg):

        if DEBUG:
            bridge = cv_bridge.CvBridge()
            cv_im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


            # plot obstacle et safe_zones

            for obs in self.obstacles.geoms:
                pts = world_to_img(obs.exterior.coords).reshape((-1,1,2))
                cv2.polylines(cv_im,[pts],True,(0,0,255))

                # if self.path.crosses(obs):
                #     coords = (obs.union(self.path)).convex_hull.exterior.coords
                #     coords = world_to_img(coords)
                #     cv2.polylines(cv_im,[coords],True,(255,0,255))

            for poly in self.polys.geoms:
                pts = world_to_img(poly.exterior.coords).reshape((-1,1,2))
                cv2.polylines(cv_im,[pts],True,(255,0,255))



            pts = world_to_img(self.terrain.exterior.coords).reshape((-1,1,2))
            cv2.polylines(cv_im,[pts],True,(0,255,0))

            im_path = world_to_img(self.path.coords)
            
            for i in range(1, im_path.shape[0]):
                # cv2.line(cv_im, self.path[i-1, :], self.path[i, :], (0, 255, 255), thickness=5)
                cv2.line(cv_im, (im_path[i-1, 0], im_path[i-1, 1]), (im_path[i, 0], im_path[i, 1]), (0, 255, 255), thickness=3)


            
            cv2.imshow("cv_im", cv_im)
            cv2.waitKey(1)

    def obj_callback(self, msg):
        A = (msg.x, msg.y)
        if not self.terrain.contains(Point(A)):
            
            cout("Objectif not in safe zone")

            #box = Point(A).buffer(2*self.l_rob)
            box = Polygon(rect_from_center(3*self.l_rob, 3*self.l_rob, A[0], A[1]))
            self.polys = self.polys.union(box)

            #pts, d = self.rec_path(self.pos, A, 0)
            I = self.terrain.exterior.intersection(box.exterior)
            if len(I.geoms) == 2:
                B = I.geoms[0]
                C = I.geoms[1]
                if cdist(np.array(A).reshape(1, -1), np.array(B).reshape(1, -1)) > cdist(np.array(A).reshape(1, -1), np.array(C).reshape(1, -1)):
                    B, C = C, B
                pts, d1 = self.rec_path(self.pos, B, 0)
                if pts != None:
                    pts.append(C)
                    self.path = LineString(pts)

            # if pts != None:
            #     I = self.terrain.exterior.intersection(box.exterior)
            #     for B in I:
            #         cout(B)
            #     self.path = LineString(pts)
        else:
            pts, d = self.rec_path(self.pos, A, 0)
            if pts != None:
                self.path = LineString(pts)

    def obstacles_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = path_planner()
    
    rclpy.spin(node)

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

