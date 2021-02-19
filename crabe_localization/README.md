# crabe_localization

Ce package contient le nécessaire pour localiser le robot à l'aide de la caméra zénithale. Un noeud utilitaire est fourni pour générer des données de calibration afin d'affiner le passage entre les systèmes de coordonnées. 

## Noeuds
***localization.py***

Ce noeud publie sur le topic */tf* la transformée entre *odom* et *base_link*.

| topics publiés  |  type |
|---|---|
| /tf  | tf2_msgs::TFMessage  |

| topics souscrits  |  type |
|---|---|
| /zenith_image  | sensor_msgs::Image  |

***calibration.py***

Ce noeud opère la même transformation que le noeud **localization**, et compare le résultat aux informations présentes  sur le topic */tf*.

  | topics souscrits  |  type |
|---|---|
| /tf  | tf2_msgs::TFMessage  |
| /zenith_image  | sensor_msgs::Image  |
