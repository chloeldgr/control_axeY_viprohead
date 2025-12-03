#include "rtJVelPlanner.h"
#include <stdlib.h>

#define PFX "rtJVelPlanner : "

JVEL_PLANNER_STRUCT JVPlanner[PLANNER_MAX_NB];
double          Planner_time = 0;
double          Planner_sampling_time = 0;
int             Planner_number = 0;

#undef fabs
#define fabs(x) ((x) >= 0.0 ? (x) : -(x))
#undef sign
#define  sgn( x )  (double)( (x) < 0.  ? -1.0 : 1.0 )
#undef pow2
#define  pow2( x )  ( ( x ) * ( x ) )

#define __DEBUG__

/**
  * @fn     rtJVelPlanner_Init
 * @brief   Procédure d'initialisation de la planification de trajectoire en vitesse articulaire
 *          permet d'initialiser certaines variables, allouer la mémoire nécessaire et de vérifier
 *          les paramètres
 * @param id
 * @param Nb
 * @param Ini
 * @param Min
 * @param Max
 * @param Velocity
 * @param Amax
 * @param Stime
 * @return
 */
int rtJVelPlanner_Init( 	int 	Nb,
                            double	*Ini,
                            double	*Min,
                            double	*Max,
                            double 	*Velocity,
                            double 	*Amax,
                            double 	Stime )
{
    int			i;

    if ( Nb > PLANNER_MAX_NB )
        return -2;
    /*
        Si on reprogramme la vitesse
        on ne change pas la vitesse courante.
    */

    if ( !Planner_number )
        for ( i = 0; i < Nb; i++ )
            JVPlanner[i].Vc = 0;

    Planner_sampling_time = Stime;
    Planner_time = 0;
    Planner_number = Nb;
    /* Initialisation des trajectoires */

    for ( i = 0; i < Nb; i++ )
    {

        /* Initialisation des objectifs + tests de validite */

        JVPlanner[i].Ini = Ini[i];
        JVPlanner[i].Min = Min[i];
        JVPlanner[i].Max = Max[i];

        if ( JVPlanner[i].Min != JVPlanner[i].Max )
        {

            /* On teste les butees */

            if ( 	( JVPlanner[i].Ini < JVPlanner[i].Min ) ||
                    ( JVPlanner[i].Ini > JVPlanner[i].Max ) )
            {
                rtJVelPlanner_Reset( );
                printf("Axe %d min : %f max : %f init : %f\n",i,JVPlanner[i].Min,JVPlanner[i].Max,JVPlanner[i].Ini );
                return -3;
            }
        }

        JVPlanner[i].Vini = JVPlanner[i].Vc;

        if ( Amax == NULL )
            JVPlanner[i].Amax = PLANNER_INFINITE_ACCEL;
        else
            JVPlanner[i].Amax = Amax[i];

        if ( JVPlanner[i].Amax <= 0 )
        {
            rtJVelPlanner_Reset( );
            return -4;
        }

        JVPlanner[i].Velocity = Velocity[i];
        JVPlanner[i].Direction = sgn( JVPlanner[i].Velocity - JVPlanner[i].Vini );

        /* Initialisation de l'instant de changement de phase */

        JVPlanner[i].t1 = fabs ( JVPlanner[i].Velocity - JVPlanner[i].Vini )/ JVPlanner[i].Amax;
    }


    return 0;
}







/***************************************************************************/
int rtJVelPlanner_Update( double *Reference )
{

    int			i, j;
    double		test;
    int			flag;

    if ( !Planner_number )
        return 1;

    /* Mise a jour du temps */

    Planner_time += Planner_sampling_time;



    /* Cas d'une trajectoire en vitesse */

    for ( i = 0; i < Planner_number; i++ )
    {

        /* Detection de proximite d'une butee */

        if ( JVPlanner[i].Min != JVPlanner[i].Max )
        {

            /* Calcul de la position future avec deceleration maximale */

            test = 	Reference[i] +
                    sgn( JVPlanner[i].Vc ) *
                    0.5 * pow2( JVPlanner[i].Vc ) / JVPlanner[i].Amax;

            if ( ( 	(	test < JVPlanner[i].Min ) ||
                    (	test > JVPlanner[i].Max ) )
                 && JVPlanner[i].Velocity )
            {

                /*
                            Redefinition de la trajectoire en vitesse :
                            arret de l'axe et reinitialisation des autres axes
                        */

                Planner_time = Planner_sampling_time;

                for ( j = 0; j < Planner_number; j++ )
                {

                    JVPlanner[j].Ini = Reference[j];

                    if ( i == j )
                        JVPlanner[j].Velocity = 0;

                    JVPlanner[j].Vini = JVPlanner[j].Vc;

                    JVPlanner[j].Direction = sgn ( JVPlanner[j].Velocity - JVPlanner[j].Vini );

                    JVPlanner[j].t1 = fabs ( JVPlanner[j].Velocity - JVPlanner[j].Vini )
                            / JVPlanner[j].Amax;
                }
            }
        }
    }

    for ( i = 0; i < Planner_number; i++ )
    {

        /* Generation de la trajectoire */

        if ( Planner_time < JVPlanner[i].t1 )
        {

            /* Phase d'acceleration */

            Reference[i] = 	JVPlanner[i].Ini +
                    JVPlanner[i].Vini * Planner_time +
                    JVPlanner[i].Direction *	(
                        0.5 * JVPlanner[i].Amax * pow2( Planner_time )
                        );

            /* Reactualisation de la vitesse courante */

            JVPlanner[i].Vc += 	JVPlanner[i].Direction *
                    JVPlanner[i].Amax *
                    Planner_sampling_time;
        }
        else
        {

            /* Vitesse courante = Vitesse de consigne */

            JVPlanner[i].Vc = JVPlanner[i].Velocity;

            /* Si la vitesse de consigne est nulle on reste en place */

            if ( JVPlanner[i].Velocity )
            {

                /* sinon on se deplace a vitesse constante */

                Reference[i] = 	JVPlanner[i].Ini +
                        JVPlanner[i].Vini * JVPlanner[i].t1 +
                        JVPlanner[i].Direction *	(
                            0.5 * JVPlanner[i].Amax * pow2( JVPlanner[i].t1 )
                            ) +
                        JVPlanner[i].Velocity * ( Planner_time - JVPlanner[i].t1 );
            }
        }

        /* Detection de butee */

        if ( JVPlanner[i].Min != JVPlanner[i].Max )
        {

            if ( Reference[i] <  JVPlanner[i].Min )
            {
                Reference[i] = JVPlanner[i].Min;
                JVPlanner[i].Vc = 0.0;
            }

            if ( Reference[i] >  JVPlanner[i].Max )
            {
                Reference[i] = JVPlanner[i].Max;
                JVPlanner[i].Vc = 0.0;
            }
        }
    }

    /* Si tous les axes sont a l'arret, on repasse en mode 0 */

    flag = 1;

    for ( i = 0; i < Planner_number; i++ )
        if ( JVPlanner[i].Vc != 0.0 )
            flag = 0;

    if ( flag )
    {
        printf("Reset JVel Planner number\n");
        Planner_number = 0;

        return 1;
    }

    return 0;
}


int rtJVelPlanner_Reset(void)
{
    printf(" Reset JVel Planner\n");
    Planner_number = 0;

    return 0;
}
