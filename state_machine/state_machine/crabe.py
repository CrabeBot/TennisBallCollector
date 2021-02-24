import tf2_ros
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Point, Pose2D
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray
from rclpy.qos import qos_profile_services_default, qos_profile_sensor_data


class Crabe():

    def __init__(self, node: Node) -> None:
        self.node = node
        
        self.waypoints = []
        self.balls = [None for i in range(10)]
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self.node)
        self.pubBackDoor = node.create_publisher(Wrench, "/force_back_door", qos_profile_services_default)
        self.pubFrontDoor = node.create_publisher(Wrench, "/force_front_door", qos_profile_services_default)
        self.pubVel = node.create_publisher(Float32, "/vel", qos_profile_services_default)
        self.pubPointA = node.create_publisher(Point, "/pointA", qos_profile_services_default)
        self.pubPointB = node.create_publisher(Point, "/pointB", qos_profile_services_default)
        self.pubTarget = node.create_publisher(Pose2D, "/objectif", qos_profile_services_default)

        node.create_subscription(Float32MultiArray, "/balls_coords", self.__balls_callback, 10)
        node.create_subscription(Float64MultiArray, "/waypoints", self.__wp_callback, qos_profile_sensor_data)
        self.newWaypoints = False


    def __balls_callback(self, msg):
        self.balls = [None for i in range(10)]
        lst = msg.data
        for i in range(len(lst)//3):
            ind = 3*i
            self.balls[int(lst[ind])] = (lst[ind+1], lst[ind+2])
        #self.node.get_logger().info(f"Balls : {self.balls}")

    def __wp_callback(self, msg):
        wpData =np.array(msg.data)
        self.waypoints = wpData.reshape((-1,2))
        self.newWaypoints = True

    def getPos(self):
        transform  = self.tfBuffer.lookup_transform('odom', 'base_link', tf2_ros.Time())
        return np.array([transform.transform.translation.x, transform.transform.translation.y])

    def getTheta(self):
        transform  = self.tfBuffer.lookup_transform('odom', 'base_link', tf2_ros.Time())
        q = transform.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def closeFrontDoor(self):
        m = Wrench()
        m.force.z = -50
        self.pubFrontDoor.publish(m)

    def openFrontDoor(self):
        m = Wrench()
        m.force.z = 50
        self.pubFrontDoor.publish(m)

    def closeBackDoor(self):
        m = Wrench()
        m.force.z = -50
        self.pubBackDoor.publish(m)

    def openBackDoor(self):
        m = Wrench()
        m.force.z = 50
        self.pubBackDoor.publish(m)

    def getBalls(self):
        return self.balls

    def setLine(self, A, B):
        m = Point()
        m.x = float(A[0])
        m.y = float(A[1])
        self.pubPointA.publish(m)
        m.x = float(B[0])
        m.y = float(B[1])
        self.pubPointB.publish(m)
        
    def setTarget(self, target):
        m = Pose2D()
        x0, y0 = target
        m.x = x0
        m.y = y0
        self.pubTarget.publish(m)

    def isIn(self):
        x, y = self.getPos()

        return (x < -6.35 and y < -13) or (x > 6.35 and y > 13)

    def setSpeed(self, v):
        m = Float32()
        m.data = v
        self.pubVel.publish(m)

    def getWaypoints(self):
        return self.waypoints