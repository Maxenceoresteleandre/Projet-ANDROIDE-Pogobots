#include "KalmanRollPitch.h"

// ETAPE D'INITIALISATION DE EKF
void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R) {
    kal->phi = 0.0f;
    kal->theta = 0.0f;

    kal->P[0] = Pinit;  kal->P[1] = 0.0f;
    kal->P[2] = 0.0f;   kal->P[3] = Pinit;

    kal->Q[0] = Q[0];   kal->Q[1] = Q[1];
    kal->R[0] = R[0];   kal->R[1] = R[1];   kal->R[2] = R[2];

}

// ETAPE DE PREDICTION
void KalmanRollPitch_Predict(KalmanRollPitch *kal, float *gyro_mesures, float T) {
    float p = gyro_mesures[0];
    float q = gyro_mesures[1];
    float r = gyro_mesures[2];
    
    int sinphi = sin(kal->phi);

    float sp = sin(kal->phi);   float cp = cos(kal->phi);
    float tt = tan(kal->theta);

    kal->phi = kal->phi + T*(p+tt*(q*sp+r*cp));
    kal->theta = kal->theta + T*(q*cp-r*sp);

    sp = sin(kal->phi);   cp = cos(kal->phi);
    float st = sin(kal->theta); float ct = cos(kal->theta); tt = st/ct;

    float A[4] = {tt*(q*cp-r*sp), (r*cp+q*sp)*(tt*tt+1.0f), 
                -(r*cp+q*sp), 0.0f};

    float Ptmp[4] = {T*(kal->Q[0] + 2.0f*A[0]*kal->P[0] + A[1]*kal->P[1] + A[1]*kal->P[2]),
                    T*(A[0]*kal->P[1] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[1]),
                    T*(A[0]*kal->P[2] + A[2]*kal->P[0] + A[1]*kal->P[3] + A[3]*kal->P[2]),
                    T*(kal->Q[1] + A[2]*kal->P[1] + A[2]*kal->P[2] + 2.0f*A[3]*kal->P[3])};

    kal->P[0] = kal->P[0] + Ptmp[0];    kal->P[1] = kal->P[1] + Ptmp[1];
    kal->P[2] = kal->P[2] + Ptmp[2];    kal->P[3] = kal->P[3] + Ptmp[3];

}

//ETAPE DE MISE A JOUR
void KalmanRollPitch_Update(KalmanRollPitch *kal, float *acc_mesures) {
    float ax = acc_mesures[0];
    float ay = acc_mesures[1];
    float az = acc_mesures[2];

    float sp = sin(kal->phi);   float cp = cos(kal->phi);
    float st = sin(kal->theta); float ct = cos(kal->theta);

    float h[3] = {g*st, -g*ct*sp, -g*ct*cp};

    float C[6] = {0.0f, g*ct, -g*cp*ct, g*sp*st, g*sp*ct, g*cp*st};

    float G[9] = {C[1]*C[1]*kal->P[3] + kal->R[0], C[2]*C[1]*kal->P[2] + C[3]*C[1]*kal->P[3], C[5]*C[1]*kal->P[3] + C[4]*C[1]*kal->P[2], 
                C[1]*(C[2]*kal->P[1] + C[3]*kal->P[3]), C[2]*(C[2]*kal->P[0] + C[3]*kal->P[2]) + C[3]*(C[2]*kal->P[1] + C[3]*kal->P[3]) + kal->R[1], C[5]*(C[2]*kal->P[1] + C[3]*kal->P[3]) + C[4]*(C[2]*kal->P[0] + C[3]*kal->P[2]),
                C[1]*(C[5]*kal->P[3] + C[4]*kal->P[1]), C[2]*(C[5]*kal->P[2] + C[4]*kal->P[0]) + C[3]*(C[5]*kal->P[3] + C[4]*kal->P[1]), C[5]*(C[5]*kal->P[3] + C[4]*kal->P[1]) + C[4]*(C[5]*kal->P[2] + C[4]*kal->P[0]) + kal->R[2]};

    float GdetInv = 1.0f/(G[0]*G[4]*G[8] - G[0]*G[5]*G[7] - G[1]*G[3]*G[8] + G[1]*G[5]*G[6] + G[2]*G[3]*G[7] - G[2]*G[4]*G[6]);

    float Ginv[9] = {GdetInv*(G[4]*G[8] - G[5]*G[7]), GdetInv*(-G[1]*G[8] + G[2]*G[7]), GdetInv*(G[1]*G[5] - G[2]*G[4]),
                    GdetInv*(-G[3]*G[8] + G[5]*G[6]), GdetInv*(G[0]*G[8] - G[2]*G[6]), GdetInv*(-G[0]*G[5] + G[2]*G[3]),
                    GdetInv*(G[3]*G[7] - G[4]*G[6]), GdetInv*(-G[0]*G[7] + G[1]*G[6]), GdetInv*(G[0]*G[4] - G[1]*G[3])};

    float K[6] = {Ginv[0]*C[1]*kal->P[1] + Ginv[3]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[6]*(C[4]*kal->P[0] + C[5]*kal->P[1]), Ginv[1]*C[1]*kal->P[1] + Ginv[4]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[7]*(C[4]*kal->P[0] + C[5]*kal->P[1]), Ginv[2]*C[1]*kal->P[1] + Ginv[5]*(C[2]*kal->P[0] + C[3]*kal->P[1]) + Ginv[8]*(C[4]*kal->P[0] + C[5]*kal->P[1]),
                Ginv[0]*C[1]*kal->P[3] + Ginv[3]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[6]*(C[4]*kal->P[2] + C[5]*kal->P[3]), Ginv[1]*C[1]*kal->P[3] + Ginv[4]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[7]*(C[4]*kal->P[2] + C[5]*kal->P[3]), Ginv[2]*C[1]*kal->P[3] + Ginv[5]*(C[2]*kal->P[2] + C[3]*kal->P[3]) + Ginv[8]*(C[4]*kal->P[2] + C[5]*kal->P[3])};

    float Ptmp[4];
    Ptmp[0] = kal->P[0]*(-K[1]*C[2] - K[2]*C[4] + 1) + kal->P[2]*(-K[0]*C[1] - K[1]*C[3] - K[2]*C[5]);  Ptmp[1] = kal->P[1]*(-K[1]*C[2] - K[2]*C[4] + 1) + kal->P[3]*(-K[0]*C[1] - K[1]*C[3] - K[2]*C[5]);
    Ptmp[2] = kal->P[0]*(-K[4]*C[2] - K[5]*C[4]) + kal->P[2]*(-K[3]*C[1] - K[4]*C[3] - K[5]*C[5] + 1);  Ptmp[3] = kal->P[1]*(-K[4]*C[2] - K[5]*C[4]) + kal->P[3]*(-K[3]*C[1] - K[4]*C[3] - K[5]*C[5] + 1);

    kal->P[0] = kal->P[0] + Ptmp[0];    kal->P[1] = kal->P[1] + Ptmp[1];
    kal->P[2] = kal->P[2] + Ptmp[2];    kal->P[3] = kal->P[3] + Ptmp[3];

    kal->phi = kal->phi+K[0]*(ax-h[0])+K[1]*(ay-h[1])+K[2]*(az-h[2]);
    kal->theta = kal->theta+K[3]*(ax-h[0])+K[4]*(ay-h[1])+K[5]*(az-h[2]);
    kal->theta = kal->theta+K[3]*(ax-h[0])+K[4]*(ay-h[1])+K[5]*(az-h[2]);

}

// FONCTIONS D'AFFICHAGE
void print_float(float i, int precision) {
  // prints a float to the console
  // precision is 10, 100, 1000... and represents the number of decimals to print
  int dec = (int)((i-(int)i) * precision);
  if (dec < 0) {
    dec = -1 * dec;
  }
  char buffer[10];
  sprintf(buffer, "%d.%d", (int)i, dec);
  printf("%8s", buffer);
}

void print_f_list(float* list, int len, int precision) {
  for (int i=0; i<len; i++) {
    print_float(list[i], precision);
    printf("\t");
  }
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Methodes pour calculer manuellement cos, sin et tan
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// mais si l'autre version du filtre de Kalman marche on n'a pas besoin de ce programme
// donc le code est pas fini
// et il le sera pas
// sauf si on se rend compte que l'autre version marche pas finalement

#define n 20
double sin(double);
double cos(double);
int main(int argc,char **argv)
{
int a,b;
double m;
double x;
printf(“Enter the degrees: “);
scanf(“%lf”, &m);
x = (3.1415926535897931*m)/180.0; //To calculate the rad

printf(“The rad of %0.2f: %f\n”,m, x);
a=sin(x); //To call the sine function
b=cos(x); //To call the cosine function
printf(“sin(%0.2lf):%lf\n”,m,sin(x));
printf(“cos(%0.2lf):%lf\n”,m,cos(x));
return 0;
}
double sin(double x) //sin calculation
{
double sum;
double fa;
double pow;
sum = 0.0;
for(int i = 0; i <= n; i++)
{
fa = 1.0;
pow = 1.0;
for(int j = 1; j <= 2*i+1; j++)
{
fa *= j;
pow *= x;
}
sum += ((i%2?-1.0:1.0)/fa)*pow;
}
return sum;
}

double cos(double x) //cosine calculation
{
double sum;
double fa;
double pow;
sum = 0.0;
for(int i = 0; i <= n; i++)
{
fa = 1.0;
pow = 1.0;
for(int j = 1; j <= 2*i; j++)
{
fa *= j;
pow *= x;
}
sum += ((i%2?-1.0:1.0)/fa)*pow;
}
return sum;
}