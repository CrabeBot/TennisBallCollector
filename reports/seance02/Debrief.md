# Debrief du 03/02

PO: Hamid Hacene


## Bilan

Pourcentage de tâches réalisées: 80 %


### Ce qui a fonctionné

- Création d'un projet Taiga

- Design de la partie mécanique du robot : 
  - Finalisation des aspects géométrique du robot : OK
  - Ajout d'un réservoir pour les balles récupérées : OK
  - Implémentation des noeuds bas niveau pour l'interface de pilotage : OK (curseurs cmd_vel pour commande manuelle)

- Traitement d'images : 
  - Récupérer les images de la caméra zénithale sous forme d'image OpenCV : OK
  - Effectuer les prétraitements nécessaires (déformations géométriques et couleurs) : OK
  - Localiser les balles de tennis et le robot : OK
  - Réaliser un tracking des balles de tennis et du robot : OK



### Ce qui n'a pas fonctionné

- Mauvaise maîtrise/utilisation de Taiga

- Implémentation des noeuds de téléopération : ajouter commande manuelle par keyboard car curseurs peu pratiques

- Tester et choisir les parties de la stack de navigation de ROS 2 qui seront utilisées la partie autonomie (path planning, costmaps, etc.) : NON FAIT (manque de temps)

- Garder l'ordre des balles lors du tracking



### Retour d'expérience du PO

- Dificulté à switcher entre gestion du projet et implémentation, à travailler.

### Conseils pour le prochain PO

- Fixer des objectifs plus SMART.

## Nouvelles mesures

- S'entraîner à utiliser Taiga et mettre à jour le projet plus régulièrement lors des prochaines séances