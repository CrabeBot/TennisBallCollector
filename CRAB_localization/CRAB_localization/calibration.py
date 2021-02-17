import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion

class calibration(Node):
    def __init__(self):
        super().__init__("saver")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.getTransform)

    def getTransform(self):
        try:
            
            transReal = self.tfBuffer.lookup_transform('odom', 'body', self.get_clock().now(), timeout=Duration(seconds=1.0))
            transImg = self.tfBuffer.lookup_transform('odom', 'base_link', self.get_clock().now(), timeout=Duration(seconds=1.0))
            self.get_logger().info(str(transImg.translation.x))
        except Exception as e:
            self.get_logger().error(str(e))


        

def main(args=None):
    rclpy.init(args=args)
    node = calibration()
    
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()