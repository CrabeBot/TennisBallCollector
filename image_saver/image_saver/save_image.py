import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
import cv2
from rclpy.qos import qos_profile_sensor_data

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class saver(Node):
    def __init__(self):
        super().__init__("saver")
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)

    def im_callback(self, msg):
        bridge = cv_bridge.CvBridge()
        cv_im = bridge.imgmsg_to_cv2(msg)
        print("affichage window")
        cv2.imshow("cv_im", cv_im)
        cv2.imwrite("image.png", cv_im)
        cv2.waitKey()
        self.spin = False
        

def main(args=None):
    rclpy.init(args=args)
    node = saver()
    
    rclpy.spin_once(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()