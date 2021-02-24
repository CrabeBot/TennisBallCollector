#!/bin/usr/env python3

from transitions import Machine, State
import rclpy
from rclpy.node import Node
import numpy as np



from state_machine.crabe import Crabe

class CrabeBotFSM(Node):

    # Définition des différents états et de leurs callback_functions
    states = [State(name='asleep', on_enter=['state0_enter_callback']),
            State(name='waiting', on_enter=['state1_enter_callback']),
            State(name='moving to a ball', on_enter=['state2_enter_callback']),
            State(name='moving to discharge area', on_enter=['state3_enter_callback']),
            State(name='dropping balls', on_enter=['state4_enter_callback'])
    ]

    def __init__(self):

        super().__init__("my_node")

        self.crabe = Crabe(self)

        self.get_logger().info("INITIALISATION OK")

        # On initialise le nombre de balles collectées dans le réservoir et la position de la balle la plus proche
        self.nb_collected_balls = 0
        self.nb_tot = 0
        self.closest_ball = np.zeros((1,2))
        self.waypoints = []
        self.waypoint_index = 0
        self.A = np.array([0, 0])
        self.B = np.array([2, 0])
        self.closest_area = 1 # 1 ou 2
        self.pos_balls = [None, None, None, None, None, None, None, None, None, None] # liste des positions des balles (liste de tuples (x,y))
        self.rate = self.create_rate(10)

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
        self.timer = self.create_timer(0.1, self.loop)
        self.statenb = 0
    # FONCTIONS DE CALLBACK À L'INTÉRIEUR DES STATES
    # ------------------------------------------------------------------------------------------------------------------------

    def state0_enter_callback(self):
        # ÉTAT INITIAL - LANCEMENT DE LA SIMULATION
        # Lorsque l'on entre dans l'état 0, on appelle directement la fonction 'wake_up' qui va trigger le state1
        self.get_logger().info("I'm in state 0 !")
        self.wake_up()
    

    def state1_enter_callback(self):
        # LE ROBOT EST EN ATTENTE DE BALLES À COLLECTER
        # On lit la liste des positions des balles sur le topic correspondant
        # Tant que la liste des balles est vide, on dort !
        # Si la liste n'est pas vide, 
        # on lit la position du robot et on calcule la position de la balle la plus proche
        # On envoie l'information au path planner qui calcule et renvoie une liste de waypoints
        # Puis on trigger le state2 en lançant self.ball_detected()

        self.cnt = 0
        self.statenb = 1
        self.get_logger().info("J'attends de détecter une balle !")

        
    
    def isCatched(self, eps):
        #same_zone = (self.closest_ball[0] - eps < self.crabe.getPos()[0] < self.closest_ball[0] + eps) and (self.closest_ball[1] - eps < self.crabe.getPos()[1] < self.closest_ball[1] + eps)
        return bool(self.crabe.getBalls()[self.closest_ball_index])#same_zone and not self.crabe.getBalls()[self.closest_ball_index]

    def loop(self):
        self.timer.cancel()
        if self.statenb == 1:
            if self.cnt < 40:
                self.cnt += 1
            else :
                if any(self.crabe.getBalls()):
                    self.get_logger().info("Une balle a été détectée")
                    self.ball_detected()

        if self.statenb==2:
            eps = 0.1
            if self.waitingFirstPath :
                if self.crabe.newWaypoints :
                    self.get_logger().info("Got first path")
                    self.crabe.newWaypoints = False
                    self.waypoint_index = 0
                    self.waypoints = self.crabe.getWaypoints()
                    self.compute_next_line()
                    self.crabe.setLine(self.A, self.B)
                    self.waitingFirstPath = False

            else :
                self.get_logger().info(str(self.crabe.getBalls()[self.closest_ball_index]))                     
                if self.isCatched(0.1):
                    self.get_logger().info("Not catched")
                    self.target = self.crabe.getBalls()[self.closest_ball_index] # Cible à atteindre
                    self.crabe.setTarget(self.target) # Envoi du point à atteindre au path_planner
                    if self.crabe.newWaypoints :
                        self.waypoint_index = 0
                        self.waypoints = self.crabe.getWaypoints() # Récupération de la liste de waypoints calculés par le path_planner
                        self.compute_next_line()

                    self.crabe.setSpeed(float(.8))
                    m = np.array(self.crabe.getPos())
                    self.get_logger().info(str(self.A)+ str(self.B))
                    if np.dot(self.B - self.A, m - self.B) > 0.0 : # Waypoint validé, on passe au suivant
                        self.compute_next_line() # Calcul de A et de B
                        self.crabe.setLine(self.A, self.B) # Envoi de ligne au contrôleur
                    # Si la position du robot est égale à la position de la balle à un epsilon près, on considère qu'ils ont la même position
                    eps = .7
                    if (self.closest_ball[0] - eps < self.crabe.getPos()[0] < self.closest_ball[0] + eps) and (self.closest_ball[1] - eps < self.crabe.getPos()[1] < self.closest_ball[1] + eps) :
                        self.crabe.openFrontDoor()

                else:
                    self.crabe.setSpeed(float(0))
                    self.nb_collected_balls += 1
                    self.nb_tot += 1

                    self.crabe.closeFrontDoor()
                    self.ball_collected()
                    self.statenb = 1
        
        self.timer = self.create_timer(0.1, self.loop)



    def state2_enter_callback(self):
        # UNE BALLE A ÉTÉ DÉTECTÉE ET LE ROBOT SE DIRIGE VERS CELLE-CI
        # Tant que la balle n'a pas été ramassée, le robot se dirige vers sa cible (la balle la plus proche)
        # Une fois que la balle a été ramassée, on update le nombre de balles collectées avec la fonction self.update_nb_balls()
        # puis on trigger le state3 ou le state1 avec self.ball_collected()
        # et l'un ou l'autre se réalisera en fonction de la condition sur le nombre de balles collectées

        self.get_logger().info("Je me dirige vers une balle !")

        eps = 0.1

        # On calcule quelle balle est la plus proche du robot : ce sera notre target
        # On enregistre l'indice associé à cette balle : pas de changement de cible en cours de route
        self.closest_ball_index = self.compute_closest_ball_index(self.crabe.getPos(), self.crabe.getBalls())
        self.closest_ball = np.array(self.crabe.getBalls()[self.closest_ball_index])

        self.target = self.crabe.getBalls()[self.closest_ball_index] # Cible à atteindre
        self.crabe.setTarget(self.target) # Envoi du point à atteindre au path_planner
        self.waypoints = self.crabe.getWaypoints() # Récupération de la liste de waypoints calculés par le path_planner
        
        self.crabe.setSpeed(float(0))
        
        self.waitingFirstPath = True


        self.get_logger().info("Waiting for first path")
        self.statenb = 2

    def state3_enter_callback(self):
        # LE NOMBRE DE BALLES DANS LE RÉSERVOIR = 4 DONC LE ROBOT SE DIRIGE VERS LA ZONE DE DÉCHARGE LA PLUS PROCHE
        # Calcule la zone la plus proche
        # Tant que le robot ne se situe pas dans la zone de décharge, il continue d'avancer (envoi de ligne au contrôleur)
        # Une fois qu'il se situe dans la zone de décharge, le state 4 est appelé avec la fonction self.is_in_discharge_area()

        self.get_logger().info("Je me dirige vers une zone de décharge !")

        # On calcule la trajectoire une seule fois : pas de changement de cible en cours de route
        self.closest_area = self.compute_closest_area(self.crabe.getPos()) # 1 ou 2
        self.target = self.compute_area_center(self.closest_area)
        self.crabe.setTarget(self.target) # Envoi du point à atteindre au path_planner
        self.rate.sleep() # On dort un peu le temps que la trajectoire soit calculée
        self.waypoints = self.crabe.getWaypoints() # Récupération de la liste de waypoints calculés par le path_planner

        while (self.crabe.isIn() == False):

            self.crabe.setSpeed(float(2))
            m = self.crabe.getPos()
            
            if np.dot(self.B - self.A, m - self.B) > 0 : # Waypoint validé, on passe au suivant
                self.compute_next_line() # Calcul de A et de B
                self.crabe.setLine(self.A, self.B) # Envoi de ligne au contrôleur
                
            self.rate.sleep()
        
        self.crabe.setSpeed(float(0))
        self.is_in_discharge_area()



    def state4_enter_callback(self):
        # LE ROBOT SE SITUE DANS UNE ZONE DE DÉCHARGE
        # Ouvre la porte arrière
        # Envoie au path planner une position à atteindre
        # Récupère la liste des waypoints
        # Tant que le robot n'est pas sorti de la zone de décharge, on envoie les lignes à suivre au contrôleur
        # Une fois qu'il est sorti de la zone, on referme la porte arrière et on revient dans l'état 1

        self.get_logger().info("Je décharge mes balles !")

        self.crabe.openBackDoor()

        # On calcule la trajectoire une seule fois : pas de changement de cible en cours de route
        self.target = self.compute_waiting_point(self.closest_area) # Cible à atteindre : un point en dehors de la zone de décharge
        self.crabe.setTarget(self.target) # Envoi du point à atteindre au path_planner
        self.waypoints = self.crabe.getWaypoints() # Récupération de la liste de waypoints calculés par le path_planner


        timer_period = 0.1  # seconds
        timer = None

        def checkIsIn():
            if not self.crabe.isIn() : 
                self.crabe.setSpeed(float(2))
                m = self.crabe.getPos()
                
                if np.dot(self.B - self.A, m - self.B) > 0 : # Waypoint validé, on passe au suivant
                    self.compute_next_line() # Calcul de A et de B
                    self.crabe.setLine(self.A, self.B) # Envoi de ligne au contrôleur
            else:
                self.crabe.setSpeed(float(0))
                self.crabe.closeBackDoor()
                self.nb_collected_balls = 0 # On remet à 0 le nombre de balles collectées dans le réservoir
                print(self.nb_collected_balls)
                print(self.nb_tot) # Le nombre total de balles ramassées n'est par contre pas remis à 0
                self.not_in_discharge_area() # On trigger l'état 1 et la boucle est bouclée
                timer.cancel()
            
        timer = self.create_timer(timer_period, checkIsIn)
        

    # ------------------------------------------------------------------------------------------------------------------------


    # AUTRES FONCTIONS
    # ------------------------------------------------------------------------------------------------------------------------

    def compute_closest_ball_index(self, pos_rob, list_balls):
        """
        Fonction prenant en entrée la position du robot et la liste des positions de toutes les balles détectées
        et retournant la balle la plus proche (calcul de distance en ligne droite)
        """
        closest_ball_index = 0
        dmin = 30
        for i in range (len(list_balls)) : 
            if list_balls[i]:
                d = np.sqrt((pos_rob[0] - list_balls[i][0])**2 + (pos_rob[1] - list_balls[i][1])**2)
                if d < dmin :
                    dmin = d
                    closest_ball_index = i

        return closest_ball_index

    
    def compute_next_line(self) :
        """
        Fonction prenant en entrée la liste des waypoints calculée par le path planner, et retournant
        la prochaine ligne [AB] à suivre, et donc à envoyer au contrôleur
        """

        if self.waypoint_index < len(self.waypoints) - 1:
            self.waypoint_index += 1
            self.A = np.array(self.waypoints[self.waypoint_index - 1])
            self.B = np.array(self.waypoints[self.waypoint_index])
    

    def compute_closest_area(self, pos_rob):
        """
        Fonction prenant en entrée la position du robot et retournant la zone de décharge la plus proche 
        (calcul de distance en ligne droite entre la position du robot et le centre de la zone de décharge)
        """

        c1 = self.compute_area_center(1)
        c2 = self.compute_area_center(2)

        d1 = np.sqrt((pos_rob[0] - c1[0])**2 + (pos_rob[1] - c1[1])**2)
        d2 = np.sqrt((pos_rob[0] - c2[0])**2 + (pos_rob[1] - c2[1])**2)

        if d1 <= d2 :
            closest_area = 1
        else :
            closest_area = 2

        return closest_area # 1 ou 2


    def compute_area_center(self, area_number):
        """
        Fonction prenant en entrée la zone de décharge (1 ou 2)
        et retournant le point target à rejoindre par le robot (au centre de la zone de décharge)
        """

        target = np.zeros((1,2))
        if area_number == 1 :
            target[0][0] = -14
            target[0][1] = -7
        elif area_number == 2 :
            target[0][0] = 14
            target[0][1] = 7

        return target


    def compute_waiting_point(self, area_number):
        """
        Fonction prenant en entrée la zone de décharge (1 ou 2)
        et retournant le point target à rejoindre par le robot après avoir déchargé les balles (sur le côté du terrain)
        """

        target = np.zeros((1,2))
        if area_number == 1 :
            target[0][0] = -13
            target[0][1] = 0
        elif area_number == 2 :
            target[0][0] = 13
            target[0][1] = 0

        return target

    # ------------------------------------------------------------------------------------------------------------------------

    
    # FONCTIONS DÉFINISSANT LES CONDITIONS DE PASSAGE À L'ÉTAT SUIVANT
    # ------------------------------------------------------------------------------------------------------------------------
    @property    

    def nb_balls_ok(self):
        res = False
        if (self.nb_collected_balls < 4) and (self.nb_tot < 8) :
            res = True
        return res


    def nb_balls_max(self):
        res = False
        if (self.nb_collected_balls == 4) or (self.nb_collected_balls == 2 and self.nb_tot == 8) :
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
