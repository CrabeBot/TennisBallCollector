import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion, Twist, Point
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_system_default

from numpy import cos, sin, arctan, arctan2, pi, cross, hstack, array, log, sign
from numpy.linalg import det, norm


class calibration(Node):
    def __init__(self):
        super().__init__("calibration")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos_profile_system_default)
        self.create_subscription(Point, "/pointA", self.getA)
        self.create_subscription(Point, "/pointB", self.getB)
        self.create_subscription(Float32, "/vel", self.getVel)

        self.a = np.array([[-3.5, 13]]).T
        self.b = np.array([[5, 13]]).T
        
        self.vel = 0

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.getTransform)


    def sawtooth(self, x):
        return (x+pi) % (2*pi)-pi
    
    def getVel(self, msg):
        self.vel = msg.data

    def getA(self, msg):
        self.a = np.array([[msg.x, msg.y]]).T
    
    def getB(self, msg):
        self.b = np.array([[msg.x, msg.y]]).T


    def control(self, x, y, yaw, a, b):
        r = .5
        m = np.array([[x, y]]).T
        phi = arctan2(b[1, 0]-a[1, 0], b[0, 0]-a[0, 0])

        # ------------ Ajout ecart a la ligne --------
        ke = 0.5
        e = det(hstack((b-a, m-a)))/norm(b-a)
        thetaBar = phi - ke*arctan(e/r)
        print(yaw, phi)
        return self.sawtooth(phi - yaw)

    def getTransform(self):
        try:
            #transReal = self.tfBuffer.lookup_transform('odom', 'body', self.get_clock().now(), timeout=Duration(seconds=1.0))
            transform  = self.tfBuffer.lookup_transform('odom', 'base_link', tf2_ros.Time())
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = arctan2(siny_cosp, cosy_cosp)

            u = self.control(transform.transform.translation.x, transform.transform.translation.y, yaw, self.a, self.b)

            m = Twist()
            m.angular.z = u
            m.linear.x = self.vel

            self.pub_cmd.publish(m)

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