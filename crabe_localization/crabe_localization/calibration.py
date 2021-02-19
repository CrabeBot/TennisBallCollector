import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion


class calibration(Node):
    def __init__(self):
        self.f = open("/home/corentin/Documents/CrabeWS/src/TennisBallCollector/crabe_localization/crabe_localization/measure.csv", 'w')
        super().__init__("calibration")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.getTransform)

    def getTransform(self):
        try:
            #transReal = self.tfBuffer.lookup_transform('odom', 'body', self.get_clock().now(), timeout=Duration(seconds=1.0))
            trans = self.tfBuffer.lookup_transform('base_link', 'body', tf2_ros.Time())
            pos  = self.tfBuffer.lookup_transform('base_link', 'odom', tf2_ros.Time())
            self.f.write("[{}, {}, {}, {}]\n".format(trans.transform.translation.x, trans.transform.translation.y, pos.transform.translation.x, pos.transform.translation.y))
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