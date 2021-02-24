 # State machine

Ce package est le cerveau du robot. 
Il est constitué de deux fichiers python : 
- crabe.py qui se charge de faire l'interface entre le haut et le bas niveau
- fsm.py qui se charge de prendre les décisions 

 # Description de la machine à état


 ## etats 

 ### ÉTAT INITIAL - LANCEMENT DE LA SIMULATION
 Lorsque l'on entre dans l'état 0, on appelle directement la fonction 'wake_up' qui va trigger le state1

 ### ETAT 0 : LE ROBOT EST EN ATTENTE DE BALLES À COLLECTER
 On lit la liste des positions des balles sur le topic correspondant
 Tant que la liste des balles est vide, on dort !
 Si la liste n'est pas vide, 
 on lit la position du robot et on calcule la position de la balle la plus proche
 On envoie l'information au path planner qui calcule et renvoie une liste de waypoints
 Puis on trigger le state2 en lançant self.ball_detected()


 ### ETAT 1 : UNE BALLE A ÉTÉ DÉTECTÉE ET LE ROBOT SE DIRIGE VERS CELLE-CI
 Tant que la balle n'a pas été ramassée, le robot se dirige vers sa cible (la balle la plus proche)
 Une fois que la balle a été ramassée, on update le nombre de balles collectées avec la fonction self.update_nb_balls()
 puis on trigger le state3 ou le state1 avec self.ball_collected()
 et l'un ou l'autre se réalisera en fonction de la condition sur le nombre de balles collectées

 ### ETAT 2 : LE NOMBRE DE BALLES DANS LE RÉSERVOIR = 4 DONC LE ROBOT SE DIRIGE VERS LA ZONE DE DÉCHARGE LA PLUS PROCHE
 Calcule la zone la plus proche
 Tant que le robot ne se situe pas dans la zone de décharge, il continue d'avancer (envoi de ligne au contrôleur)
 Une fois qu'il se situe dans la zone de décharge, le state 4 est appelé avec la fonction self.is_in_discharge_area()


 ### ETAT 3 : LE ROBOT SE SITUE DANS UNE ZONE DE DÉCHARGE
 Ouvre la porte arrière
 Envoie au path planner une position à atteindre
 Récupère la liste des waypoints
 Tant que le robot n'est pas sorti de la zone de décharge, on envoie les lignes à suivre au contrôleur
 Une fois qu'il est sorti de la zone, on referme la porte arrière et on revient dans l'état 1

 ## Transitions

 [State 0 --> State 1] : Robot asleep --> Robot en attente de balles à ramasser
 Condition de passage : le robot est démarré

 [State 1 --> State 2] : En attente de balles à ramasser --> Se dirige vers la balle la plus proche
 Condition de passage : une balle a été détectée et le nombre de balles dans le réservoir est inférieur à 4

 [State 2 --> State 1] : Se dirige vers la balle la plus proche --> En attente de balles à ramasser
 Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est inférieur à 4

 [State 2 --> State 3] : Se dirige vers la balle la plus proche --> Se dirige vers la zone de décharge la plus proche
 Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est égal à 4

 [State 3 --> State 4] : Se dirige vers la zone de décharge la plus proche --> Décharge les 4 balles dans la zone
 Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est égal à 4

 [State 4 --> State 1] : Décharge les 4 balles dans la zone --> En attente de balles à ramasser
 Condition de passage : une balle a été ramassée et le nombre de balles dans le réservoir est égal à 4