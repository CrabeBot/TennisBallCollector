# Objectifs du 24/02

PO: Corentin Lemoine


## Fonctionnalités attendues

- Détecter et suivre les balles pour décider quand une balle a disparu
- path_finder communiquant avec le reste, intégration des joueurs
- classe Crabe pour factoriser le code de la fsm
- Fichiers launch


## Tâches à réaliser

--> Détection / Tracking des balles : 
        - Bugfix, suivi des balles qui ont disparu

--> FSM :
        - Validation de waypoints
        - Suivi des balles ramassées
        - Choix de la prochaine balle
        - Dépôt des balles
        - Classe Crabe

--> Fichier launch général

--> Test et débug

## Challenges techniques

- Tracking
- fonction récursive pour path_finder
- fsm : défintion des fonctions, robustesse
