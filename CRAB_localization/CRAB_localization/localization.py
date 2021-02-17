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

from tf2_msgs.msg import TFMessage

class localizer(Node):
    def __init__(self):
        super().__init__("saver")
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)

        self.br = tf2_ros.TransformBroadcaster(self)
        #self.tf_publisher = self.create_publisher(msg_type=TFMessage, topic="/tf", qos_profile=self.profile)
        


    
    

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

    def imgCoordToTF(self, ix, iy):
        return (
            -0.02392*ix + 8.56684846, 
            (iy-639.98)/(-41.842)
            )

    def im_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        cv_im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        im_hsv = cv2.cvtColor(cv_im, cv2.COLOR_BGR2HSV)
        lh = 44
        hh = 99
        ls = 113
        hs = 255
        lv = 76
        hv = 255
        green_im = cv2.inRange(im_hsv, (lh, ls, lv), (hh, hs, hv))
        red_im = cv2.inRange(im_hsv, (0, ls, lv), (12, hs, hv))
        
        gcX, gcY, rcX, rcY = 0,0,0,0
        green_cnts, hierarchy = cv2.findContours(green_im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in green_cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            gcX = int(M["m10"] / M["m00"])
            gcY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.circle(cv_im, (gcX, gcY), 2, (255, 255, 0), -1)

        red_cnts, hierarchy = cv2.findContours(red_im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in red_cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            rcX = int(M["m10"] / M["m00"])
            rcY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.circle(cv_im, (rcX, rcY), 2, (255, 0, 0), -1)
        #we suppose we found it

        robotx = (rcX + gcX) / 2
        roboty = (rcY + gcY) / 2
        robotTheta = np.arctan2(rcX-gcX, rcY - gcY) - np.pi/2
        cv2.circle(cv_im, (int(robotx), int(roboty)), 5, (255, 0, 255), -1)
        cv2.arrowedLine(
            cv_im, 
            (int(robotx), 
            int(roboty)), 
            (int(robotx) - int(20*np.sin(robotTheta)), int(roboty) - int(20*np.cos(robotTheta))),
            (0,0,0)
        )

        cv2.imshow("cv_im", cv_im)
        cv2.waitKey(1)

        rx, ry = self.imgCoordToTF(roboty, robotx)

        t = geometry_msgs.msg.TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = rx
        t.transform.translation.y = ry
        t.transform.translation.z = 0.0
        q = self.rpy2Quaternion(0, 0, robotTheta)
        t.transform.rotation = q

        #tf_msg = TFMessage()
        #tf_msg.transforms = [t]
        #self.tf_publisher.publish(tf_msg)
        self.br.sendTransform(t)
        

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