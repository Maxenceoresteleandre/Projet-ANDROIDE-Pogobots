Expériences sur les Pogobots, Loona Macabre, Master ANDROIDE, Mai 2023

Fichiers relatifs aux expériences concernant la communication:
- Nombre de messages reçus et voisins perçus par un robot (receiver) en fonction de la fréquence d'émission de message
- Nombre de messages reçus et voisins perçus par un robot en fonction de son temps de sonde

Receiver contient le code à exécuter sur le robot test: regarder sa messagerie, envoyer ou non un message, calculer le nombre de messages qu'il a reçu au cours de l'expérience et l'afficher. 
Senders contient le code à exécuter sur les autres robots: regarder sa messagerie puis envoyer ou non un message.
Le message envoyé contient 100o de pay load utile.

Les robots senders exécutent leur code infiniment, tandis que le robot receiver s'arrête au bout d'un temps de sondes.

Les expériences sont faites sur 3 robots: il y a donc 2 robots senders et 1 robot receiver.

L'expérience consiste à faire sonder le robot pendant un certain temps puis afficher le nombre de messages reçus dans la console, et recommencer en changeant le paramètre étudié. L'affichage dans la console doit être réécrit dans un fichier (dans le répertoire data). Une fois qu'on a effectué le nombre d'expériences souhaité, on appelle les fichiers python correspondant pour tracer les graphiques

Protocole:
- CHANGEMENT DE LA FREQUENCE D'EMISSION DE MESSAGES:
    - dans senders/main.c: mettre la fréquence à tester à la ligne 27 #define FREQEMISSION
    - dans receiver/main.c: idem ligne 30
- CHANGEMENT DU TEMPS DE SONDE:
    - dans receiver/main.c: changer la ligne 31 #define TEMPSSONDE avec le temps de sonde à tester
- faire exécuter le code à chaque robot, en branchant le robot receiver en dernier et en le gardant branché pour les affichages console
- recopier les résultats dans les fichiers data à lire par le python
- recommencer en changeant les paramètres testés
- dans un terminal, appeler les fichiers python pour tracer les graphes

Informations sur ce que j'ai fait: 
- Pour les 10 expériences répétées, j'ai écrit le fichier tempsSonde10Exp.py qui trace les différents graphes (courbes, boxplots et moyenne+écart type) d'évolution du nombre de voisins et de messages en fonction du temps de sonde. Sinon, j'ai juste fait une seule expérience par fréquence/temps de sonde testé et plot les graphes résultant dans freqEmission.py et tempsSonde1Exp.py.
- A chaque fois j'écrit les affichages à la main dans les fichiers data car je n'ai pas pu écrire directement dans les fichiers depuis receiver/main.c. Ma manière de faire est assez laborieuse
- répertoire data:
    - dataFreqAvecClean.txt: nombre de messages reçus en vidant la pile à chaque passage dans le while. ligne i = nombre de messages avec la fréquence i
    - dataFreqSansClean.txt: nombre de messages reçus sans vider la pile à chaque passage dans le while. ligne i = nombre de messages avec la fréquence i
    - dataSonde.txt: nombre de messages reçus et de voisins perçus en fonction du temps de sonde. ligne i = nombre de messages et voisins avec le temps de sonde i
    - dataSondei.txt: nombre de messages reçus et de voisins perçus au temps de sonde = i secondes. ligne j = nombre de messages et voisins avec le temps de sonde i à l'expérience j