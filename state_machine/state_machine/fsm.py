#!/bin/usr/env python3

from transitions import Machine, State
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float32MultiArray

from crabe import Crabe

class CrabeBotFSM(Node):

    # Définition des différents états et de leurs callback_functions
    states = [State(name='asleep', on_enter=['state0_enter_callback']),
            State(name='waiting', on_enter=['state1_enter_callback']),
            State(name='moving to a ball', on_enter=['state2_enter_callback'], on_exit=['state2_exit_callback']),
            State(name='moving to discharge area', on_enter=['state3_enter_callback']),
            State(name='dropping balls', on_enter=['state4_enter_callback'])
    ]

    def __init__(self):

        super().__init__("my_node")

        self.crabe = Crabe(self)

        self.get_logger().info("INITIALISATION OK")

        #self.balls_pos_subscriber = self.create_subscription(Float32MultiArray, '/balls_pos', self._callback_balls_pos, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) # À CORRIGER EN FONCTION DU TOPIC ET DU MSG
        #self.rob_pos_subscriber = self.create_subscription(Pose, '/rob_pos', self._callback_rob_pos, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) # À CORRIGER EN FONCTION DU TOPIC ET DU MSG
        #self.closest_ball_publisher = self.create_publisher(Pose, '/closest_ball', 10) # À CORRIGER EN FONCTION DU TOPIC ET DU MSG
        #self.force_avant_publisher = self.create_publisher(Wrench, '/force_porte_avant', 10) # À CORRIGER EN FONCTION DU TOPIC ET DU MSG
        #self.force_arriere_publisher = self.create_publisher(Wrench, '/force_porte_arriere', 10) # À CORRIGER EN FONCTION DU TOPIC ET DU MSG

        # On initialise le nombre de balles collectées dans le réservoir et la position de la balle la plus proche
        self.nb_collected_balls = 0
        self.closest_ball = Pose()

        # Initialisation de la machine à états
        self.machine = Machine(model=self, states=CrabeBotFSM.states, initial='asleep')

        # AJOUT DES TRANSITIONS
        # ------------------------------------------------------------------------------------------------------------------------

        # [State 0 --> State 1] : Robot asleep --> Robot en attente de balles à ramasser
        # Condition de passage : le robot est démarré
        self.machine.add_transition(trigger='wake_up', source='asleep', dest='waiting')

        # [State 1 --> State 2] : En attente de balles à ramasser --> Se dirige vers la balle la plus proche
        # Condition de passage : une balle a été détectée et le nombre de balles dans le réservoir est inférieur à 4
        self.machine.add_transition(trigger='ball_detected', source='waiting', dest='moving to a ball')


        # [State 2 --> State 1] : Se dirige vers la balle la plus proche --> En attente de balles à ramasser
        # Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est inférieur à 4
        self.machine.add_transition(trigger='ball_collected', source='moving to a ball', dest='waiting', conditions=['nb_balls_ok'])


        # [State 2 --> State 2] : Se dirige vers la balle la plus proche --> Se dirige vers la balle la plus proche
        # REFLEXIVE TRANSITION
        # Condition de passage : une balle encore plus proche a été détectée
        self.machine.add_transition(trigger='closer_ball_detected', source='moving to a ball', dest='=')


        # [State 2 --> State 3] : Se dirige vers la balle la plus proche --> Se dirige vers la zone de décharge la plus proche
        # Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est égal à 4
        self.machine.add_transition(trigger='ball_collected', source='moving to a ball', dest='moving to discharge area', conditions=['nb_balls_max'])


        # [State 3 --> State 4] : Se dirige vers la zone de décharge la plus proche --> Décharge les 4 balles dans la zone
        # Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est égal à 4
        self.machine.add_transition(trigger='is_in_discharge_area', source='moving to discharge area', dest='dropping balls')


        # [State 4 --> State 1] : Décharge les 4 balles dans la zone --> En attente de balles à ramasser
        # Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est égal à 4
        self.machine.add_transition(trigger='not_in_discharge_area', source='dropping balls', dest='waiting')

        # ------------------------------------------------------------------------------------------------------------------------



    # FONCTIONS DE CALLBACK DES TOPICS ROS
    # ------------------------------------------------------------------------------------------------------------------------
    def _callback_balls_pos(self, msg):     
        self.list_balls = msg


    def _callback_rob_pos(self, msg):
        self.pos_rob = msg
        
        # À COMPLÉTER
        # If self.state == 3 and pos_rob est dans la zone de décharge :
        # Appeler le trigger de l'état3 vers l'état4 : self.is_in_discharge_area()
        # Else if self.state == 4 and pos_rob est en dehors de la zone de décharge :
        # Appeler le trigger de l'état4 vers l'état1 : self.not_in_discharge_area()
    # ------------------------------------------------------------------------------------------------------------------------



    # FONCTIONS DE CALLBACK À L'INTÉRIEUR DES STATES
    # ------------------------------------------------------------------------------------------------------------------------

    def state0_enter_callback(self):
        # Lorsque l'on entre dans l'état 0, on appelle directement la fonction 'wake_up' qui va trigger le state1
        self.get_logger().info("I'm in state 0 !")
        self.wake_up()
    

    def state1_enter_callback(self):
        # On lit la liste des positions des balles sur le topic correspondant
        # Tant que la liste des balles est vide, on dort !
        # Si la liste n'est pas vide, 
        # on lit la position du robot et on calcule la position de la balle la plus proche
        # puis on trigger le state2 en lançant self.ball_detected()

        self.get_logger().info("I'm in state 1 !")

        while len(self.crabe.getBalls()) == 0:
            # AJOUTER LIGNE DE CODE POUR SLEEP UN PETIT PEU

        self.closest_ball = compute_closest_ball(self.crabe.getPos(), self.crabe.getBalls())
        #self.closest_ball_publisher.publish(Pose(closest_ball)) # On publie sur le topic lu par le path_planner
        self.ball_detected()

    
    def state2_enter_callback(self):
        # On lit la liste des positions des balles
        # Si on détecte une nouvelle balle plus proche que celle qu'on allait chercher, 
        # on trigger à nouveau le state2 en lançant self.closer_ball_detected()
        # Sinon on trigger le state3 ou le state1 avec self.ball_collected() et l'un ou l'autre se réalisera en fonction
        # de la condition

        self.get_logger().info("I'm in state 2 !")

        closest_ball_new = self.compute_closest_ball(self.crabe.getPos(), self.crabe.getBalls())
        if closest_ball_new != self.closest_ball :
            self.closer_ball_detected()
        
        # TROUVER UN MOYEN POUR RESTER (OU BOUCLER) SUR LE STATE 2 TANT QUE LA BALLE N'EST PAS COLLECTÉE
        # À COMPLÉTER



    def state2_exit_callback(self):
        # Update le nombre de balles collectées avec la fonction self.update_nb_balls() quand on passe à
        # l'état 1 ou 3 (mais par contre il ne faut pas qu'elle se lance quand on boucle sur l'état2)

        # FONCTION À COMPLÉTER
        self.get_logger().info("I leave state 2 !")
    

    def state3_enter_callback(self):
        # Calcule la zone la plus proche
        # Envoie au contrôleur un waypoint ?

        self.get_logger().info("I'm in state 3 !")

        # FONCTION À COMPLÉTER
    

    def state4_enter_callback(self):
        # Ouvre la porte arrière (publie sur le topic ros une force)
        # Envoie au contrôleur un waypoint pour sortir de la zone ?

        self.get_logger().info("I'm in state 4 !")

        # FONCTION À COMPLÉTER
    # ------------------------------------------------------------------------------------------------------------------------




    # AUTRES FONCTIONS
    # ------------------------------------------------------------------------------------------------------------------------
    def update_nb_balls(self):
        self.nb_collected_balls += 1


    def compute_closest_ball(self, pos_rob, list_balls):
        closest_ball = Pose()
        # FONCTION À COMPLÉTER

        return closest_ball
    # ------------------------------------------------------------------------------------------------------------------------

    
    # FONCTIONS DÉFINISSANT LES CONDITIONS DE PASSAGE À L'ÉTAT SUIVANT
    # ------------------------------------------------------------------------------------------------------------------------
    @property    

    def nb_balls_ok(self):
        res = False
        if self.collected_balls < 4 :
            res = True
        return res


    def nb_balls_max(self):
        res = False
        if self.collected_balls == 4 :
            res = True
            return res
    # ------------------------------------------------------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = CrabeBotFSM()
    node.wake_up()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
