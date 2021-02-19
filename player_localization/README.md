# player_localization

Ce package contient le nécessaire pour localiser ls joueurs à l'aide de la caméra zénithale. Un noeud utilitaire est fourni pour générer des données de calibration afin d'affiner le passage entre les systèmes de coordonnées. 

## Noeuds
***p_localization.py***

Ce noeud publie la position des joueurs sur 2 topics différents. Le *theta* des messages publiés contient la direction actuelle des joueurs.

| topics publiés  |  type |
|---|---|
| /player1Pose  | geometry_msgs::Pose2D  |
| /player2Pose  | geometry_msgs::Pose2D  |

| topics souscrits  |  type |
|---|---|
| /zenith_image  | sensor_msgs::Image  |

***p_calibration.py***

Ce noeud opère la même transformation que le noeud **p_localization**, et compare le résultat aux informations présentes  sur le topic */link_states*.

  | topics souscrits  |  type |
|---|---|
| /link_states  | gazebo_msgs::LinkStates  |
| /zenith_image  | sensor_msgs::Image  |