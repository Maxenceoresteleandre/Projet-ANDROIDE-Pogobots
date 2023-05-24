## auteur: Loona Macabre
# plot du nombre de messages reçus en fonction de la fréquence d'émission de message
# les mesures ont été prises sur 100 itérations (cf receiver/main.c)
# pour des fréquences de 0 à 100% du temps

import matplotlib.pyplot as plt


f1 = open("data/dataFreqAvecClean.txt")
f2 = open("data/dataFreqSansClean.txt")
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

plt.plot(x, y, label="sans vider la liste de messages à chaque itération")


plt.xlabel("Fréquence d'émission en moyenne (%)")
plt.ylabel("Nombre de messages reçus")

plt.title("Nombre de messages reçus en fonction de la fréquence d'émission \n(conversation entre 3 robots, fréquence de tick: 30Hz)")
plt.legend()
plt.show()