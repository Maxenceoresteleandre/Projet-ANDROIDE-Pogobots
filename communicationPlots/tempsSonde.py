## auteur: Loona Macabre
# plot du nombre de messages reçus en fonction du temps de sonde
# les mesures ont été prises sur 100 itérations (cf receiver/main.c)
# pour des fréquences de tick de 10 à 100Hz

import matplotlib.pyplot as plt


f1 = open("dataSondeAvecClean.txt")
f2 = open("dataSondeSansClean.txt")
l1 = f1.readlines()
f1.close()
l2 = f2.readlines()
f2.close()

x = []
y = []
for seq in l1:
    seqBis = seq.split()
    
    x.append(int(seqBis[0]))
    y.append(int(seqBis[1]))
plt.plot(x, y, label="en vidant la liste de messages à chaque itération")

x = []
y = []
for seq in l2:
    seqBis = seq.split()
    
    x.append(int(seqBis[0]))
    y.append(int(seqBis[1]))

plt.plot(x, y, label="sans la liste de messages à chaque itération")


plt.xlabel("Nombre de ticks par seconde")
plt.ylabel("Nombre de messages reçus")

plt.title("Nombre de messages reçus en fonction du temps de sonde (conversation entre 3 robots, freq d'émission: 50%)")
plt.legend()
plt.show()