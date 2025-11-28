#ifndef __RT_TELEOP_MODULE_H__
#define __RT_TELEOP_MODULE_H__
#include <stdbool.h>

/**
  * Structure de données utilisée pour la communication téléopération
  */
#define TELEOP_WAIT_MODE    0
#define TELEOP_SYNCHR_MODE  1

/* Attention structure pour 1 ddl*/
typedef struct{
    bool    bIsRunning;
    unsigned int mode;
    double  slave_position;
    double  master_position;
    double  slave_force;
    double  master_force;
}TELEOPERATION_STRUCT;

#endif
