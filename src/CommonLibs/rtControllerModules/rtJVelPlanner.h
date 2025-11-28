#ifndef __RT_JVEL_PLANNER_H__
#define __RT_JVEL_PLANNER_H__

#define PLANNER_MAX_NB          10
#define PLANNER_INFINITE_ACCEL	1e300	/* Acceleration infinie */

/* Définition des modes de fonctionnement*/


typedef struct	{
    double          Ini;						/* Position initiale */
    double          Min;						/* Butee inferieure de position */
    double          Max;						/* Butee superieure de position */
    double          Amax;						/* Acceleration maximale */
    double          Vini;						/* Vitesse initiale */
    double          Velocity;					/* Vitesse desiree */
    double          Direction;					/* Sens de deplacement */
    double          Vc;                        /* Vitesse courante */
    double          t1;						/* Instant de changement de trajectoire */
}JVEL_PLANNER_STRUCT;


int rtJVelPlanner_Init(     	int 	Nb,
                                double	*Ini,
                                double	*Min,
                                double	*Max,
                                double 	*Velocity,
                                double 	*Amax,
                                double 	Stime );

int rtJVelPlanner_Update( double *Reference );
int rtJVelPlanner_Reset( void);
#endif
