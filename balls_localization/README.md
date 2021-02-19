# Tennis Court

Package ROS 2 pour la détectioin et localisation des balles.


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

Inclure le fichier:
```python
def generate_launch_description():
    tennis_court_share = get_package_share_directory("tennis_court")
    tennis_court_launch_file = os.path.join(tennis_court_share, "launch", "tennis_court.launch.py")
    tennis_court_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(tennis_court_launch_file)
    )
    return LaunchDescription([tennis_court_launch])
```

- Lancer le noued de détection : 

Lancer le fichier:
```shell
ros2 launch balls_localization ball_tracking.launch.py
```

### Nœuds

- `b_localization.py (localizer)`  

### Topics

- `/balls_xy` (*MultiArray ?*)  
  Liste des balles.

### Paramètres


