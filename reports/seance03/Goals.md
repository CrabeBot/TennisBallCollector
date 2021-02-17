# Objectifs du 17/02

PO: Romane FLÉCHARD


## Fonctionnalités attendues

- Détecter et tracker de manière automatique et suffisamment précise les balles et le robot, malgré les obstacles
- Robot fonctionnel pouvant être téléopéré pour récupérer les balles (sans en perdre !)
- Avoir une stratégie robuste de déplacement du robot pour le ramassage des balles et l'évitement des obstacles


## Tâches à réaliser

--> Détection / Tracking des balles :
      - Intégrer le code dans un noeud ROS
      - Prendre en compte la disparition des balles
      - Créer une liste contenant la position des balles et tenant compte de leur ordre d'apparition
      - Publier sur un topic la position de la première balle de la liste (celle à aller chercher en priorité)

--> Architecture du robot :
      - Adapter la porte à l'arrière du robot pour le déchargement des balles
      - Créer une porte à l'avant du robot pour empêcher la perte de balles lors d'une décélération

--> Localisation du robot :
      - Filtrer les positions (Kalman ou simple lissage)
      - Corriger les distorsions

--> Navigation / Contrôle du robot :
      - Ajouter commande manuelle du robot par keyboard pour tests Gazebo
      - Faire schéma architecture ROS du projet et définir les communications entre les différents noeuds
      - Créer noeud ROS de navigation (uniquement la structure pour l'instant)
      - Définir stratégie de déplacement du robot pour le ramassage des balles
      - Définir stratégie d'évitement d'obstacles (statiques et dynamiques)


## Challenges techniques

- Meilleure utilisation de Taïga
- Création de portes coulissantes dans l'architecture du robot sous Gazebo
- Adapter le projet pour prendre en compte la nouvelle spécification : savoir éviter les joueurs (obstacles dynamiques)
