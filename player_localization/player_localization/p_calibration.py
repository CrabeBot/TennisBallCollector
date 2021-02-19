import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo

from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.node import Node

import numpy as np

import cv2
import cv_bridge

from gazebo_msgs.msg import LinkStates


class localizer(Node):
    def __init__(self):
        super().__init__("player_localizer")

        self.profile = qos_profile_sensor_data
        self.im_subscriber = self.create_subscription(Image, "/zenith_camera/image_raw", self.im_callback, qos_profile=self.profile)

        self.pl_subscriber = self.create_subscription(LinkStates, "/link_states", self.pl_callback, qos_profile=self.profile)

        self.bridge = cv_bridge.CvBridge()

        self.dp1 = np.array([0.0, 0.0])
        self.dp2 = np.array([0.0, 0.0])
        self.tp1 = np.array([0.0, 0.0])
        self.tp2 = np.array([0.0, 0.0])

        self.f = open("/home/corentin/Documents/CrabeWS/src/TennisBallCollector/player_localization/measures.csv", "w")

        self.cnt = 0
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.writeToFile)

    def writeToFile(self):
        self.cnt += 1
        print(self.cnt)
        self.f.write("{}, {}, {}, {}\n".format(self.dp1[0],self.dp1[1],self.tp1[0],self.tp1[1]))
        self.f.write("{}, {}, {}, {}\n".format(self.dp2[0],self.dp2[1],self.tp2[0],self.tp2[1]))

    def pl_callback(self, msg):
        index = msg.name.index('player_1::player_link')
        self.tp1[0] = msg.pose[index].position.x
        self.tp1[1] = msg.pose[index].position.y
        
        index = msg.name.index('player_2::player_link')
        self.tp2[0] = msg.pose[index].position.x
        self.tp2[1] = msg.pose[index].position.y

    def imToWorldCoord(self, x, y):
        cx = x - 640
        cy = y - 360
        wx = -cy * 0.025
        wy = -cx * 0.025

        ang = np.arctan2(-wy, -wx)
        norm = np.sqrt(wx**2 + wy**2)
        wx += 0.24098204379027707 * norm * np.cos(ang)
        wy += 0.24098204379027707 * norm * np.sin(ang)

        return np.array([wx, wy])

    def im_callback(self, msg):
        try:
            cv_im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            im_hsv = cv2.cvtColor(cv_im, cv2.COLOR_BGR2HSV)
            lh = 0
            hh = 45
            ls = 1
            hs = 255
            lv = 0
            hv = 55

            left_im = cv2.inRange(im_hsv[:, :640], (lh, ls, lv), (hh, hs, hv))
            right_im = cv2.inRange(im_hsv[:, 640:], (lh, ls, lv), (hh, hs, hv))
            
            left_cx, left_cy, right_cx, right_cy = 0,0,0,0

            left_moments = cv2.moments(left_im)
            left_cx = int(left_moments["m10"] / left_moments["m00"])
            left_cy = int(left_moments["m01"] / left_moments["m00"])
            cv2.circle(cv_im, (left_cx, left_cy), 6, (255, 255, 0), -1)
            
            self.dp2 = self.imToWorldCoord(left_cx, left_cy)

            right_moments = cv2.moments(right_im)
            right_cx = int(right_moments["m10"] / right_moments["m00"])
            right_cy = int(right_moments["m01"] / right_moments["m00"])
            cv2.circle(cv_im, (640 + right_cx, right_cy), 6, (255, 0, 0), -1)
            self.dp1 = self.imToWorldCoord(640 + right_cx, right_cy)

        except Exception as e:
            print("Robot not found : " + str(e))
            
        cv2.imshow("cv_im2", cv_im)
        cv2.waitKey(1)

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