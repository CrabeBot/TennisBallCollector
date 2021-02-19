# Debrief du 17/02

PO: Romane FLÉCHARD


## Bilan

Pourcentage de tâches réalisées: 65 %


### Ce qui a fonctionné

--> Détection / Tracking des balles : 
        - Intégrer le code dans un noeud ROS : OK
        - Prendre en compte la disparition des balles : Ok 

--> Architecture du robot :
        - Adapter la porte à l'arrière du robot pour le déchargement des balles : OK
        - Créer une porte à l'avant du robot pour empêcher la perte de balles lors d'une décélération : OK

--> Localisation du robot :
        - Filtrer les positions (Kalman ou simple lissage) : OK
        - Corriger les distorsions : quasi OK

--> Navigation / Contrôle du robot :
        - Faire schéma architecture ROS du projet et définir les communications entre les différents noeuds : OK
        - Définir stratégie de déplacement du robot pour le ramassage des balles et l'évitement d'obstacles : OK


### Ce qui n'a pas fonctionné

--> Détection / Tracking des balles :
        - Créer une liste contenant la position des balles et tenant compte de leur ordre d'apparition
        - Publier sur un topic la position de la première balle de la liste (celle à aller chercher en priorité)
        CAUSE : Problème de numérotation lorsque deux balles passent à côté l'une de l'autre

--> Navigation / Contrôle du robot :
        - Ajouter commande manuelle du robot par keyboard pour tests Gazebo
        CAUSE : abandon de l'idée car trop compliqué à mettre en place pour pas grand chose

        - Commencer à coder la commande du robot : 
            - Envoi de la position des obstacles sur un topic
            - Envoi de la position des balles sur un topic
            - Création d'une fsm
            - Création d'un path_planner
            - Création d'un noeud contrôleur
        CAUSE : Manque de temps mais c'est en cours. À poursuivre la prochaine fois.


### Retour d'expérience du PO

Intéressant de passer du temps sur la gestion de projet.
Beaucoup de temps "perdu" à régler des bugs dans le code, qu'il faudrait anticiper lorsqu'on fixe les objectifs.
On a toujours tendance à se fixer des objectifs trop élevés, car on se place dans des conditions idéales, sans prévoir les aléas qui créent des pertes de temps. Il faudrait penser à réduire la quantité d'objectifs pour un temps donné, quitte à en ajouter plus tard s'ils sont réalisés plus vite que prévu.


### Conseils pour le prochain PO

Continuer les efforts pour mettre à jour le projet Taïga en continu.


## Nouvelles mesures

