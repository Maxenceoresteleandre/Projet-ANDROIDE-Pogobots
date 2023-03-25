#include <stdio.h>
#include <stdlib.h>

typedef struct maStruct 
{
    int a;
    int b;
    int c[3];
} maStruct;


void test(maStruct *s) {
    s->a = 18;  s->b = 20;
    s->c[0] = 4;     s->c[1] = 5;     s->c[2] = 6;
}

void testPrint(maStruct *s){
    printf("a+b = %d", s->a+s->b);
}

int main(void) {
    maStruct s;
    maStruct *sptr = &s;
    sptr->a = 3;    sptr->b = 9;    sptr->c[0] = 1;     sptr->c[1] = 2;     sptr->c[2] = 3;

    printf("s->a = %d, s->b = %d\n", sptr->a, sptr->b);
    printf("s->c = [");
    for (int i=0;i<3;i++){
        printf("%d ", sptr->c[i]);
    }
    printf("]\n");
    test(sptr);
    printf("s->a = %d, s->b = %d\n", sptr->a, sptr->b);
    printf("s->c = [");
    for (int i=0;i<3;i++){
        printf("%d ", sptr->c[i]);
    }
    printf("]\n");


    testPrint(sptr);
    return 1;
}