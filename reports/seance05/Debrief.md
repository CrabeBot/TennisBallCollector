# Debrief du 24/02

PO: Corentin Lmeoine


## 1. Bilan

Pourcentage de tâches réalisées: 50 %


### 2. Ce qui a fonctionné

#### Détection / Tracking des balles : 
        - Tracking des balles
        - Récupération des infos par la FSM

#### FSM :
        - classe intermédiaire Crabe
        - implémentation des fonctions pour chaque état

#### Localisation du robot :
        - Publication /tf

#### Localisation des joueurs:
        - Publication /topic

#### Navigation / Contrôle du robot :
        - V1 du path_finder : ok
        - gestion obstacles, safe_zone: ok
        - suiveur de ligne : ok


### 3. Ce qui n'a pas fonctionné

#### Détection / Tracking des balles :
        - Mieux gerer les disparitions des balles
        - Detection de la capture d'une balle peu précise
        
#### FSM :
        - Bugs lié au garbage collector
        - Trop de tests en même temps, pas assez de progressivitié




### Retour d'expérience du PO

J'ai beaucoup codé, ce qui enfreint le PO dans son rôle.
-> La période de bugfix est la plus complexe à gérer car c'est le moment où chacun est plongé dans sa partie, mais tout le monde a besoin de petites informations rapidement.
-> Le temps est difficile à gérer car on ne peut pas anticiper l'ampleur des bugs, et leur nombre.
-> Comme Colin, trouver le temps de faire le PO + le code !!
 
### Conseils pour le prochain PO

Toujours continuer les efforts pour mettre à jour le projet Taïga en continu.
Il faut être plus itératif et demander des versions qui vont fournir des données pour que les autres puissent tester, même si les données ne sont pas fiables/factices (ex : balles).

