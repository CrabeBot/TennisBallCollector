import tf2_ros
import numpy as np

class Crabe():

    def __init__(self, node) -> None:
        self.node = node
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        transform  = self.tfBuffer.lookup_transform('odom', 'base_link', tf2_ros.Time())
        q = transform.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        u = self.control(transform.transform.translation.x, transform.transform.translation.y, yaw, self.a, self.b)

    def getPos(self):
        pass

    def getTheta(self):
        pass

    def closeFrontDoor(self):
        pass

    def openFrontDoor(self):
        pass

    def closeBackDoor(self):
        pass

    def openBackDoor(self):
        pass

    def getBalls(self):
        pass

    def setTarget(self, A, B):
        pass

    def isIn(self):
        pass

    def setSpeed(self, v):
        pass