# Objectifs du 03/02

PO: Hamid HACENE


## Fonctionnalités attendues

- Robot fonctionnel pouvant être téléopéré pour récupérer les balles ;
- Détection automatique et tracking des balles et du robot ;
- Premières pistes de reflexion sur la partie autonomie (stack de navigation ROS2).

## Tâches à réaliser

- Créer un projet Taiga ;

- Design de la partie mécanique du robot : 
  - Finalisation des aspects géométrique du robot ;
  - Implémentation des noeuds bas niveau pour l'interface de pilotage ; 
  - Implémentation des noeuds de téléopération ;

- Traitement d'images : 
  - Récuperer les images de la caméra zénithale sous forme d'image OpenCV ;
  - Effectuer les prétraitements nécessaires (déformations géométriques et couleurs) ;
  - Localiser les balles de tennis et le robot ;
  - Réaliser un tracking des balles de tennis et du robot ;

- Tester et choisir les parties de la stack de navigation de ROS 2 qui seront utiliser la partie autonomie (path planning, costmaps, etc.).

## Challenges techniques

- Trouver un moyen de régler la QoS sous ROS2 ;
- Explorer la stack de navigation très peu documéntée pour l'instant ;
