import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge
import cv2
from rclpy.qos import qos_profile_sensor_data

import rclpy
from rclpy.node import Node


class saver(Node):
    def __init__(self):
        super().__init__("saver")
        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)
        self.img = 0

    def im_callback(self, msg):
        if self.img <= 100:
            bridge = cv_bridge.CvBridge()
            print("Got image")
            print(msg.encoding)
            cv_im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(cv_im.shape)
            cv2.imshow("cv_im", cv_im)
            cv2.imwrite("/home/corentin/Pictures/image_ball_{0}.png".format(str(self.img)), cv_im)
            print("/home/corentin/Pictures/image_ball_{0}.png".format(str(self.img)))
            cv2.waitKey(1)
            self.img += 1
            

def main(args=None):
    rclpy.init(args=args)
    node = saver()
    
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()