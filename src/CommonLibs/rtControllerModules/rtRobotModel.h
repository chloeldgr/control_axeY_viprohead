#ifndef RT_ROBOT_MODEL_H
#define RT_ROBOT_MODEL_H
#include <math.h>

typedef struct{
    double H[4][4]; // matrice homogène

}ROBOT_MODEL_STRUCT;

int rtDirectKinematicsModel(double *input, double *output);
int rtInverseKinematicsModel(double *input, double *output);


#endif
