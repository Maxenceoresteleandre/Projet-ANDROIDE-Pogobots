import matplotlib.pyplot as plt
import numpy as np


TEST = 1 # 1: sans moteur; 2: moteur à 512; 3: moteur à 1023

for TEST in [1, 2, 3]:
    if TEST == 1:
        f = open("imu_data.txt")
    elif TEST == 2:
        f = open("imu_motor512.txt")
    elif TEST == 3:
        f = open("imu_motor1023.txt")
    else: 
        print("TEST incorrect")

    l = f.readlines()
    f.close()

    iter = np.arange(1,len(l)//2+1)

    accX = []
    accY = []
    accZ = []
    gyroX = []
    gyroY = []
    gyroZ = []

    for seq in l:
        seqBis = seq.split()
        # formatage:
        # seqBis = [sensor, x, coordX, y, coordY, z, coordZ] => len = 7

        # ACCELERATION READ
        if seqBis[0] == 'Acc':
            accX.append(int(seqBis[2])/1000)
            accY.append(int(seqBis[4])/1000)
            accZ.append(int(seqBis[6])/1000)
        # GYROSCOPE READ
        elif seqBis[0] == 'Gyro':
            gyroX.append(int(seqBis[2])/100)
            gyroY.append(int(seqBis[4])/100)
            gyroZ.append(int(seqBis[6])/100)
        else:
            print("fichier corrompu")




    # plot
    pltAcc = plt.subplot(211)
    pltAcc.plot(iter, accX, label="acc X")
    pltAcc.plot(iter, accY, label="acc Y")
    pltAcc.plot(iter, accZ,label="acc Z")
    pltAcc.set_xlabel("timestep")
    pltAcc.set_ylabel("acc (m/s^2)")

    titre = "Données de l'IMU BRUTES"
    if TEST == 1:
        titre+=" (sans moteur)"
    elif TEST == 2:
        titre+=" (moteur m à 512)"
    elif TEST == 3:
        titre+=" (moteur m à 1023)"
    pltAcc.set_title(titre)
    pltAcc.legend()

    pltGyro = plt.subplot(212)
    pltGyro.plot(iter, gyroX, label="gyro X")
    pltGyro.plot(iter, gyroY, label="gyro Y")
    pltGyro.plot(iter, gyroZ, label="gyro Z")
    pltGyro.set_xlabel("timestep")
    pltGyro.set_ylabel("gyro (rads)")
    pltGyro.set_yticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi], [r'$-\pi$', r'$-\frac{ \pi }{ 2 }$', '$0$', r'$\frac{ \pi }{ 2 }$',
                                                        r'$\pi$'])
    pltGyro.legend()



    plt.show()