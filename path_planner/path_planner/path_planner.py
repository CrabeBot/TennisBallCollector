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

from tf2_msgs.msg import TFMessage


from shapely.geometry import Polygon, LineString, MultiPolygon

DX, DY = 30, 35

WIDHT = 1280 - 2 * DX
HEIGHT = 720 - 2 * DY

DEBUG = True

def rect(l, L, x=0, y=0):
    return np.array(((x, y), (x+l, y), (x+l, y+L), (x, y+L)))

def rect_from_center(l, L, x=0, y=0):
    return np.array(((x-l/2, y-L/2), (x+l/2, y-L/2), (x+l/2, y+L/2), (x-l/2, y+L/2)))

def world_to_img(X):
    return (X + np.array(((DX + WIDHT/2, DY + HEIGHT/2)))).astype(int)

def img_to_world(x, y):
    pass


class path_planner(Node):
    def __init__(self):
        super().__init__("wps")
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)
        self.obj_subscriber = self.create_subscription(Pose2D, "/objectif", self.obj_callback, qos_profile=self.profile)

        self.br = tf2_ros.TransformBroadcaster(self)

        self.l_rob = 0.5
        self.obstacles_fixes()
        self.terrain = Polygon(rect_from_center(WIDHT, HEIGHT))
        

        self.pos = np.array((-300, 0)).reshape(1, -1)
        self.path = LineString(((0, 0), (0, 0)))
        
    def obstacles_fixes(self):

        l_box, L_box = 110, 90

        l_filet, L_filet = 10, 520

        self.obstacles = MultiPolygon([
            Polygon(rect(l_box, L_box, -WIDHT/2, -HEIGHT/2)), 
            Polygon(rect(l_box, L_box, WIDHT/2 - l_box, HEIGHT/2 - L_box)), 
            Polygon(rect_from_center(l_filet, L_filet))
            ])

    def compute_path(self, obj):
        self.path = LineString((self.path, obj))
        

    def im_callback(self, msg):

        if DEBUG:
            bridge = cv_bridge.CvBridge()
            cv_im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


            # plot obstacle et safe_zones

            for obs in self.obstacles.geoms:
                pts = world_to_img(obs.exterior.coords).reshape((-1,1,2)).astype(int)
                cv2.polylines(cv_im,[pts],True,(0,0,255))

            pts = world_to_img(self.terrain.exterior.coords).reshape((-1,1,2)).astype(int)
            cv2.polylines(cv_im,[pts],True,(0,255,0))

            im_path = world_to_img(self.path.coords)
            print(im_path)
            for i in range(1, self.path.shape[0]):
                # cv2.line(cv_im, self.path[i-1, :], self.path[i, :], (0, 255, 255), thickness=5)
                cv2.line(cv_im, (im_path[i-1, 0], im_path[i-1, 1]), (im_path[i, 0], im_path[i, 1]), (0, 255, 255), thickness=3)


            # plot lines
            
            cv2.imshow("cv_im", cv_im)
            cv2.waitKey(1)

    def obj_callback(self, msg):
        A = np.array((msg.x, msg.y)).reshape(1, -1)
        self.compute_path(A)


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

