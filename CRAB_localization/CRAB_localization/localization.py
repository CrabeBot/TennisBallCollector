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
from geometry_msgs.msg import Quaternion
import cv2
import cv_bridge

class localizer(Node):
    def __init__(self):
        super().__init__("saver")
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)

        self.map2odomTF = geometry_msgs.msg.TransformStamped()
        
        self.map2odomTF.header.stamp = self.get_clock().now().to_msg()
        self.map2odomTF.header.frame_id = "map"
        self.map2odomTF.child_frame_id = "odom"
        self.map2odomTF.transform.translation.x = 0.0
        self.map2odomTF.transform.translation.y = 0.0
        self.map2odomTF.transform.translation.z = 0.0
        q = self.rpy2Quaternion(0, 0, 0)
        self.map2odomTF.transform.rotation = q
    
    

    def rpy2Quaternion(self, roll, pitch, yaw):

        # Abbreviations for the various angular functions
        cy = np.cos(yaw * 0.5);
        sy = np.sin(yaw * 0.5);
        cp = np.cos(pitch * 0.5);
        sp = np.sin(pitch * 0.5);
        cr = np.cos(roll * 0.5);
        sr = np.sin(roll * 0.5);

        q = Quaternion();
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q


    def im_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        print("Got image")
        print(msg.encoding)
        cv_im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        im_hsv = cv2.cvtColor(cv_im, 'BGR2HSV')
        lh = 44
        hh = 99
        ls = 113
        hs = 255
        lv = 76
        hv = 255
        green_im = cv2.inRange(im_hsv, (lh, ls, lv), (hh, hs, hv))
        red_im = cv2.inRange(im_hsv, (0, ls, lv), (12, hs, hv))
        
        green_circles = cv2.HoughCircles(green_im, cv2.HOUGH_GRADIENT, 1, 10,
                               param1=5, param2=30,
                               minRadius=1, maxRadius=30)
        red_circles = cv2.HoughCircles(red_im, cv2.HOUGH_GRADIENT, 1, 10,
                               param1=5, param2=30,
                               minRadius=1, maxRadius=30)
    
    
        if red_circles is not None:
            circles = np.uint16(np.around(red_circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(cv_im, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(cv_im, center, radius, (255, 0, 255), 3)
        if green_circles is not None:
                    circles = np.uint16(np.around(green_circles))
                    for i in circles[0, :]:
                        center = (i[0], i[1])
                        # circle center
                        cv2.circle(cv_im, center, 1, (0, 100, 100), 3)
                        # circle outline
                        radius = i[2]
                        cv2.circle(cv_im, center, radius, (255, 0, 255), 3)
        print(cv_im.shape)
        print("affichage window")
        cv2.imshow("cv_im", cv_im)
        cv2.waitKey(1)

        robotx, roboty, robotTheta = 0,0,0
        
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = rclpy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = robotx
        t.transform.translation.y = roboty
        t.transform.translation.z = 0.0
        q = self.rpy2Quaternion(0, 0, robotTheta)
        t.transform.rotation = q
    
        self.map2odomTF.header.stamp = self.get_clock().now().to_msg()
        br.sendTransform(self.map2odomTF)
        br.sendTransform(t)
        

def main(args=None):
    rclpy.init(args=args)
    node = localizer()
    
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()