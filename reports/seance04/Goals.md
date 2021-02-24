# Objectifs du 19/02

PO: Colin Baumgard


## Fonctionnalités attendues

- Détecter et tracker de manière automatique et suffisamment précise les balles et le robot, malgré les obstacles
- Robot fonctionnel
- path_finder robuste


## Tâches à réaliser

--> Détection / Tracking des balles : 
        - Passage en coordonnées monde
        - Tracking envoyé sur un topic 

--> FSM :
        - Squelette FSM
        - Code "vide" FSM 

--> Localisation du robot :
        - Publication /topic

--> Localisation des joueurs:
        - Publication /topic

--> Navigation / Contrôle du robot :
        - V1 du path_finder : ok
        - gestion obstacles, safe_zone: ok
        - suiveur de ligne : ok

--> Détection / Tracking des balles :
        - Mieux gerer les disparition des balles

--> Navigation / Contrôle du robot :
        - ajouter code dans fsm


## Challenges techniques

- Tracking
- fonction récursive pour path_finder
- fsm : prise en main et définition
