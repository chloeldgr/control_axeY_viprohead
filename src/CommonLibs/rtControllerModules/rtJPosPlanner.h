#ifndef RTJPOSPLANNER_H
#define RTJPOSPLANNER_H

#define PLANNER_MAX_NB			10		/* Nombre maximum d'axes */
#define PLANNER_INFINITE_ACCEL	1e300	/* Acceleration infinie */


typedef struct	{
    double	Ini;						/* Position initiale */
    double	Fin;						/* Position finale */
    double	Min;						/* butee inferieure */
    double	Max;						/* butee superieure */
    double	Amax;						/* Acceleration maximale */
    double	Vmax;						/* Vitesse maximale */
    char	Type;						/* 0 = /\, 1 = /-\ */
    double	Tc;							/* Temps de cycle */
    double	Delta;						/* Varaition angulaire */
    double	Direction;					/* Sens de deplacement */
    double	Vc;							/* Vitesse maximale ajustee */
    double	Ac;							/* Acceleration maximale ajustee */
    double	t1, t2;						/* Instants de changement de trajectoire */
    } JPOS_PLANNER_STRUCT;

int rtPPlanner_Init( int, double*, double*, double*, double*, double*, double*, double );
int rtPPlanner_Update( double* );
int rtPPlanner_Reset( void );
#endif
