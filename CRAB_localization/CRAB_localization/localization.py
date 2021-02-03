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
        print(cv_im.shape)
        print("affichage window")
        cv2.imshow("cv_im", cv_im)
        cv2.imwrite("/home/corentin/Documents/CrabeWS/src/TennisBallCollector/image_saver/image_ball_2.png", cv_im)
        cv2.waitKey()

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