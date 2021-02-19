# crabe_controller

Ce package contient le noeud qui opère le robot à bas niveau. 

## Noeuds
***controller.py***

Ce noeud publie sur le topic */cmd_vel* la commande qui permet de faire suivre au robot la ligne (A, B) où A et B sont reçus respectivement sur les topics */pointA* et */pointB*. La vitesse du robot est définie par la valeur reçue sur le topic */vel*.

| topics publiés  |  type |
|---|---|
| /cmd_vel  | geometry_msgs::Twist  |

| topics souscrits  |  type |
|---|---|
| /tf  | tf2_msgs::TFMessage  |
| /pointB  | geometry_msgs::Point  |
| /pointA  | geometry_msgs::Point  |
| /vel  | std_msgs::Float32  |
