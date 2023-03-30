import matplotlib.pyplot as plt
import numpy as np


# 1: ligne droite; 2: tourne à droite; 3: demi tour (à gauche)
for TEST in [1, 2, 3]:
    if TEST == 1:
        f1 = open("../data/imu_straight_noMotor.txt")
        f2 = open("../data/imu_straight_512lr.txt")
        f3 = open("../data/imu_straight_1023lr.txt")
    elif TEST == 2:
        f1 = open("../data/imu_turnRight_noMotor.txt")
        f2 = open("../data/imu_turnRight_512lr.txt")
        f3 = open("../data/imu_turnRight_1023lr.txt")
    elif TEST == 3:
        f1 = open("../data/imu_demiTour_noMotor.txt")
        f2 = open("../data/imu_demiTour_512lr.txt")
        f3 = open("../data/imu_demiTour_1023lr.txt")
    else: 
        print("TEST incorrect")

    l1 = f1.readlines()
    l2 = f2.readlines()
    l3 = f3.readlines()
    f1.close()
    f2.close()
    f3.close()



    for l in [l1, l2, l3]:
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
        iter = np.arange(1,len(l)//2+1)

        pltAcc = plt.subplot(211)
        pltAcc.plot(iter, accX, label="acc X")
        pltAcc.plot(iter, accY, label="acc Y")
        pltAcc.plot(iter, accZ,label="acc Z")
        pltAcc.set_xlabel("timestep")
        pltAcc.set_ylabel("acc (m/s^2)")

        titre = "Données BRUTES de l'IMU"
        if TEST == 1:
            titre+=" - ligne droite"
        elif TEST == 2:
            titre+=" - tourne à droite"
        else:
            titre+=" - demi tour"

        if l == l1:
            titre+=" (sans moteur)"
        elif l == l2:
            titre+=" (moteur l,r à 512)"
        else:
            titre+=" (moteur l,r à 1023)"
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