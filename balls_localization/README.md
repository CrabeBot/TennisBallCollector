# balls_localization

Ce package contient le nécessaire pour localiser les balles à l'aide de la caméra zénithale. Les coordonées de celles-ci sont publiées selon leurs ordre d'apparition (ou presque !). Il permet de tracker les balles, de gérer le cas des balles disparus et des nouvelles balles qui apparaissent.


## Fonctionnalités

Ce package permet de :
- Détecter et tracker les balles ;
- Publier sur un *topic* la liste des coordonnées des balles (dans l'ordre d'apparition).


## Utilisation

- Démarrer la simulation en lançant ou incluant le fichier [`tennis_court.launch.py`](launch/tennis_court.launch.py).

Lancer le fichier:
```shell
ros2 launch tennis_court tennis_court.launch.py
```

- Insérer la ligne suivante dans un fichier launch:
```python
ball_tracking = Node(
        package="balls_localization",
        #condition=IfCondition(LaunchConfiguration("manager")),
        #parameters=[{"use_sim_time": True}],
        output="screen",
        #emulate_tty=True,
        executable="b_localizer"
    )
```

- Ou bien lancer le noued : 

Lancer le fichier:
```shell
ros2 run balls_localization b_localizer
```


### Nœuds

***b_localization.py (b_localizer)***  


### Topics

| topics publiés  |  type |
|---|---|
| /balls_coords  | std_msgs::Float32MultiArray  |

| topics souscrits  |  type |
|---|---|
| /zenith_image  | sensor_msgs::Image  |


### Paramètres
- Possibilité de visualiser les traitements (variable self.visualize à modifier dans le code source).

