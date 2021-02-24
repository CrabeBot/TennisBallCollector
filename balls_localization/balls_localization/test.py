import rclpy
from rclpy.node import Node


import numpy as np
import geometry_msgs
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32, Float32MultiArray


class test(Node):
    def __init__(self):
        super().__init__("test")
        self.balls_subscriber = self.create_subscription(Float32MultiArray, "/balls_coords", self.coords_callback, 10)

        self.dico = {
            0:[],
            1:[],
            2:[],
            3:[],
            4:[],
            5:[],
            6:[],
            7:[],
            8:[],
            9:[],
            }

    def coords_callback(self, msg):
        list_coords = msg.data
        n = int(len(list_coords)/2)
        for i in range(n):
            self.dico[i] = [list_coords[2*i], list_coords[2*i+1]]
        self.get_logger().info(f"{n-1} balles trouvees")
        self.get_logger().info(f"{self.dico}")
        self.get_logger().info("======================================")

    def disappeared_callback(self, msg):
        index = msg.data
        n = len(index)
        for i in range(n):
            self.dico[i] = [10000 , 10000]
        #self.get_logger().info(f"{n-1} balles trouvees")
        #self.get_logger().info(f"{self.dico}")
        #self.get_logger().info("======================================")

def main(args=None):
    rclpy.init(args=args)
    
    node = test()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()