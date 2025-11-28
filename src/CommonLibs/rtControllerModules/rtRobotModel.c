#include "rtRobotModel.h"

ROBOT_MODEL_STRUCT robot_model;
/**
 * @brief rtDirectKinematicsModel
 * @param input : coordonnées articulaires
 * @param output : coordonnées catésiennes
 * @return code d'erreur
 */
int rtDirectKinematicsModel(double *input, double *output)
{
    int i;
    double q1, q2;
    double zf[3];
    /* récupération des coordonnées articulaires*/
    q1 = input[0];
    q2 = input[1];

    /* Remplissage de la matrice homogène*/
    /* 1ère colonne*/
    robot_model.H[0][0] = cos(q1)*cos(q2);
    robot_model.H[0][1] = sin(q1)*cos(q2);
    robot_model.H[0][2] = sin(q2);
    robot_model.H[0][3] = 0;

    /* 2ème colonne*/
    robot_model.H[1][0] = sin(q1);
    robot_model.H[1][1] = -cos(q1);
    robot_model.H[1][2] = 0;
    robot_model.H[1][3] = 0;

    /* 3ème colonne*/
    robot_model.H[2][0] = cos(q1)*sin(q2);
    robot_model.H[2][1] = sin(q1)*sin(q2);
    robot_model.H[2][2] = -cos(q2);
    robot_model.H[2][3] = 0;

    /* 4ème colonne*/
    robot_model.H[3][0] = 0;
    robot_model.H[3][1] = 0;
    robot_model.H[3][2] = 0;
    robot_model.H[3][3] = 1;

    /*Vecteur directeur de l'axe d'insertion*/
    for (i = 0 ; i < 3;i++)
        zf[i] = robot_model.H[2][i];

    output = zf;
    return 0;
}


/**
 * @brief rtInverseKinematicsModel
 * @param input : coordonnées cartésiennes
 * @param output : coordonnées articulaires
 * @return  code d'erreur
 */
int rtInverseKinematicsModel(double *input, double *output)
{

    output[0] = arctan2(input[1], input[0]);

    output[1] = arccos(-input[2]);

    return 0;
}
