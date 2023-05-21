## auteur: Loona Macabre
# plot du nombre de messages reçus en fonction de la fréquence d'émission de message
# les mesures ont été prises sur 100 itérations (cf receiver/main.c)
# pour des fréquences de 0 à 100% du temps

import matplotlib.pyplot as plt


# f = open("dataFreq.txt")
f = open("dataFreq2.txt")
l = f.readlines()
f.close()

x = []
y = []
for seq in l:
    seqBis = seq.split()
    
    x.append(int(seqBis[0]))
    y.append(int(seqBis[1]))

plt.plot(x, y)
plt.xlabel("Fréquence d'émission (%)")
plt.ylabel("Nombre de messages reçus")

plt.title("Nombre de messages reçus en fonction de la fréquence d'émission (conversation entre 3 robots, fréquence de tick: 30Hz)")
plt.show()