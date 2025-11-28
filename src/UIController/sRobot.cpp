#include "sRobot.h"



sRobot::sRobot(void):
    nb_joints(4),
    nb_forces(1),
    nb_dof(3),
    bIsCal(false),
    bIsPowerOn(false)
{
    int i;
    // Allocation
    JPos    = new double[nb_joints];
    Forces  = new double[nb_forces];
    // Initialisation des variables
    for (i= 0; i < nb_joints;i++)
        JPos[i] =0;
    for (i = 0; i < nb_forces;i++)
        Forces[i] =0;
}


sRobot::~sRobot(void)
{
}



void sRobot::setJPos(double * joint_pos)
{
    int i;
    for (i = 0 ; i < nb_joints; i++)
        JPos[i] = joint_pos[i];
}
