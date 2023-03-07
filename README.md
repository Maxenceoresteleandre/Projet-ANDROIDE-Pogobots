# Projet-ANDROIDE-Pogobots

Objectifs:
- Trouver un moyen de corriger la trajectoire en ligne droite d'un Pogobot:
    * soit dynamiquement à partir des données de l'IMU (filtre de Kalman, compliqué)
    * soit à la main en calibrant les moteurs

- Etudier comment les robots communiquent:
    * Envoi et réception de signaux IR
    * Dialogue vs communication de groupe

- Implémenter des comportements:
    * exploration au hasard (chaque pogobot explore indépendemment des autres)
    * détection de bordure (les pogobots n'ayant pas de voisins sur au moins un coté changent de couleur)
    * phototaxie (aggrégation autour d'une source lumineuse, qu'on pourra déplacer au cours du temps)
    * aggrégation (se rapprocher les uns des autres)
    * suivi de leader (un pogobot explore, les autres suivent en fil indienne)
    * boucle infinie (les pobobots forment une boucle, puis se suivent en fil indienne - il s'agit d'une variation du précédent)
    * déplacement en ligne frontale (les pogobots avancent en ligne et corrige leur trajectoire pour rester aligné)
    * dispersion (les pogobots s'écartent les uns des autres)


OU ON EN EST:
- Données de l'IMU:
    * Bruit très intense lorsqu'on allume les moteurs, plus la puissance des moteurs est haute, plus il y a de bruit
    * Comment implémenter un filtre de Kalman prenant en compte tous ces paramètres:
        * 3 moteurs en tout sur un pogobot, pas forcément tous allumés, pas au même moment etc
        * l'idée de base était d'utiliser l'IMU pour calibrer les moteurs afin de suivre une trajectoire en ligne droite. Hors, si on modifie dynamiquement la puissance des moteurs, le bruit est lui aussi changé => filtre de Kalman doit être modifié en conséquence... cycle vicieux non ?
    * idée: calibrer à la main le pogobot, soit via l'api, soit avec un verre (élaborer le protocole)