# Projet-ANDROIDE-Pogobots

Ce dépot est consacré à notre contribution au projet P-ANDROIDE Pogobots portant sur la robotique en essaim.

OBJECTIFS du projet:
- Trouver un moyen de corriger la trajectoire en ligne droite d'un Pogobot:
    * soit dynamiquement à partir des données de l'IMU (implémentation d'un filtre de Kalman)
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


NOTRE CONTRIBUTION:
- Au niveau du déplacement:
   - Etude de la vitesse de déplacement d'un robot
   - Implémentation d'un filtre de Kalman sur les données de l'IMU
   - Etude des effets du filtre pour plusieurs trajectoires sur robot déplacé à la main, moteurs droit et gauche à différentes puissances
   - Correction de la trajectoire en effectuant un cablibrage sur les moteurs à l'aide des données passées au filtre Kalman
- Au niveau de la communication:
   - Etude du rayon de communication entre deux robots
   - Etude de la capacité de communication en prenant en compte les collisions de messages et le temps de sonde
   - Implémentation de la détection de bordure

Liens des vidéos:
- Calibrage des robots: https://www.youtube.com/watch?v=7Rjwod2-fSE
- Envoi de message avec une règle en plexiglas: https://youtube.com/shorts/I_IfwiTpmD0?feature=share
- Détection de bordure: https://youtube.com/shorts/tM0YwDOTcRI?feature=share