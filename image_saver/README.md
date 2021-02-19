# image_saver

Ce package contient un noeud qui permet d'enregistrer les images de la caméra zénithale pour expérimenter avec les différents traitements d'image. 

## Noeuds
***saver***

Ce noeud enregistre les images reçues. Le chemin d'enregistrement des fichiers est *hard-codé* dans le script.

| topics souscrits  |  type |
|---|---|
| /zenith_image  | sensor_msgs::Image  |