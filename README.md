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