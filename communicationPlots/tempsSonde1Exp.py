## auteur: Loona Macabre
# plot du nombre de messages reçus et de voisins perçus en fonction du temps de sonde
# de 0 à 1 seconde avec un pas de 0.1 seconde

import matplotlib.pyplot as plt


f = open("data/dataSonde.txt")
l = f.readlines()
f.close()

x = []
y = []
for seq in l:
    seqBis = seq.split()
    
    x.append(float(seqBis[0]))
    y.append(int(seqBis[1]))
plt.subplot(2, 1, 1)
plt.plot(x, y)
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de messages")
plt.title("Nombre de messages reçus en fonction du temps de sonde \n(conversation entre 4 robots, freq d'émission: 50%, freq de tick: 30Hz)")

x = []
y = []
for seq in l:
    seqBis = seq.split()
    
    x.append(float(seqBis[0]))
    y.append(int(seqBis[2]))
plt.subplot(2, 1, 2)
plt.plot(x, y)
plt.xlabel("Temps de sonde (s)")
plt.ylabel("Nombre de voisins")
plt.title("Nombre de voisins perçus en fonction du temps de sonde \n(conversation entre 4 robots, freq d'émission: 50%, freq de tick: 30Hz)")

plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.4,
                    hspace=0.6)
plt.show()