import matplotlib.pyplot as plt
import numpy as np


f01 = open("data/dataSonde01.txt")
f02 = open("data/dataSonde02.txt")
f03 = open("data/dataSonde03.txt")
f04 = open("data/dataSonde04.txt")
f05 = open("data/dataSonde05.txt")
f06 = open("data/dataSonde06.txt")
f07 = open("data/dataSonde07.txt")
f08 = open("data/dataSonde08.txt")
f09 = open("data/dataSonde09.txt")
f1 = open("data/dataSonde1.txt")

F = [f01, f02, f03, f04, f05, f06, f07, f08, f09, f1]

temps = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
nbMsg = np.zeros((10, 10))
nbVoisins = np.zeros((10, 10))

mVoisins = []
stdVoisins = []
mMsg = []
stdMsg = []

for i in range(len(F)):
    fi = F[i]
    l = fi.readlines()

    nbMsgi = []
    nbVoisinsi = []
    for seq in l:
        seqBis = seq.split()
        
        nbMsgi.append(float(seqBis[0]))
        nbVoisinsi.append(int(seqBis[1]))

    nbMsg[i] = nbMsgi
    nbVoisins[i] = nbVoisinsi
    nbMsgi = np.array(nbMsgi)
    nbVoisinsi = np.array(nbVoisinsi)
    mMsg.append(nbMsgi.mean())
    mVoisins.append(nbVoisinsi.mean())
    stdMsg.append(nbMsgi.std())
    stdVoisins.append(nbVoisinsi.std())


### MEAN ET STD
plt.subplot(121)
plt.plot(temps, mVoisins, label="Moyenne")
plt.plot(temps, stdVoisins, label="Ecart type")
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de voisins")
plt.legend()
plt.title("Moyenne et écart type sur 10 expériences \ndu nombre de voisins perçus en fonction\n du temps de sonde (conversation entre 4 robots, \nfreq d'émission: 50%, freq de tick: 30Hz)")

plt.subplot(122)
plt.plot(temps, mMsg, label="Moyenne")
plt.plot(temps, stdMsg, label="Ecart type")
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de messages")
plt.legend()
plt.title("Moyenne et écart type sur 10 expériences \ndu nombre de messages reçus en fonction\n du temps de sonde (conversation entre 4 robots, \nfreq d'émission: 50%, freq de tick: 30Hz)")

plt.show()


### BOXPLOTS
plt.subplot(122)
plt.boxplot(nbMsg.T)
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de messages")
plt.title("Nombre de messages reçus en fonction du temps de sonde\n sur 10 expériences (conversation entre 4 robots,\n freq d'émission: 50%, freq de tick: 30Hz)")

plt.subplot(121)
plt.boxplot(nbVoisins.T)
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de voisins")
plt.title("Nombre de voisins perçus en fonction du temps de sonde\n sur 10 expériences (conversation entre 4 robots,\n freq d'émission: 50%, freq de tick: 30Hz)")

plt.show()


### COURBES
plt.subplot(121)
i=1
for x in nbVoisins.T:
    plt.plot(temps, x, label="Expérience n°"+str(i))
    i+=1
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de voisins")
plt.legend()
plt.title("Nombre de voisins perçus en fonction du temps de sonde\n sur 10 expériences (conversation entre 4 robots,\n freq d'émission: 50%, freq de tick: 30Hz)")

plt.subplot(122)
i=1
for x in nbMsg.T:
    plt.plot(temps, x, label="Expérience n°"+str(i))
    i+=1
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de messages")
plt.legend()
plt.title("Nombre de messages reçus en fonction du temps de sonde\n sur 10 expériences (conversation entre 4 robots,\n freq d'émission: 50%, freq de tick: 30Hz)")

plt.show()
