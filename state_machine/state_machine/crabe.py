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
        """
        Lit la position du robot sur le topic correspondant et la renvoie au format np.array([[x, y]])
        """
        pass

    def getTheta(self):
        pass


    def openFrontDoor(self):
        """
        Publie sur le topic correspondant le couple à appliquer pour ouvrir la porte avant
        """
        pass

    def closeFrontDoor(self):
        """
        Publie sur le topic correspondant le couple à appliquer pour fermer la porte avant
        """
        pass


    def openBackDoor(self):
        """
        Publie sur le topic correspondant le couple à appliquer pour ouvrir la porte arrière
        """
        pass


    def closeBackDoor(self):
        """
        Publie sur le topic correspondant le couple à appliquer pour fermer la porte arrière
        """
        pass

    def getBalls(self):
        """
        Lit la liste des positions des balles détectées sur le topic correspondant 
        et la renvoie au format np.array([[x1, y1], [x2, y2], ...])
        """
        pass


    def setTarget(self, target):
        """
        Récupère la cible (au format np.array([[x, y]])) calculée par la fsm
        et l'envoie au path planner via le topic correspondant
        """
        pass
    

    def getWaypoints(self) :
        """
        Lit la liste des waypoints calculés par le path_planner sur le topic correspondant 
        et la renvoie au format np.array([[x1, y1], [x2, y2], ...])
        """
        pass


    def setLine(self, A, B):
        """
        Récupère les points A et B (au format np.array([[x, y]])) calculés par la fsm
        et les envoie au contrôleur via le topic correspondant
        """
        pass


    def isCatched():
        """
        Fonction retournant True si la balle target a été ramassée, False sinon.
        On considère que la balle est ramassée si elle ne fait plus partie de la liste des balles détectées
        """

    def isIn(self):
        """
        Fonction retournant True si le robot se situe dans une des deux zones de décharge, False sinon.
        """
        pass


    def setSpeed(self, v):
        pass