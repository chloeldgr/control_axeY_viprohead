#include "rtJPosPlanner.h"
#include <math.h>
#include <stdlib.h>

#define PFX "rtJPosPlanner :"

JPOS_PLANNER_STRUCT PPlanner[PLANNER_MAX_NB];

double          planner_time = 0;
double          planner_sampling_time = 0;
int             planner_number = 0;

#undef fabs
#define fabs(x) ((x) >= 0.0 ? (x) : -(x))
#undef sign
#define  sgn( x )  (double)( (x) < 0.  ? -1.0 : 1.0 )
#undef pow2
#define  pow2( x )  ( ( x ) * ( x ) )


/**
 * @fn      int rtPPlanner_Init(int, double*, double*, double*, double*, double*,double*,double*)
 * @brief   Initialisation de la trajectoire position articulaire
 * @param   Nb    : Nombre d'axes
 * @param   Ini   : Position initiale
 * @param   Fin   : Position Finale
 * @param   Min   : Butée Inférieure
 * @param   Max   : Butée Supérieure
 * @param   Amax  : Accélération Max si Amax = NULL accélération Infinie
 * @param   Vmax  : Vitesse maximale
 * @param   Stime : Période d'échantillonnage
 * @return  Code d'erreur
 */
int rtPPlanner_Init( 	int		Nb,
                        double	*Ini,
                        double	*Fin,
                        double	*Min,
                        double	*Max,
                        double	*Amax,
                        double	*Vmax,
                        double	Stime )
{

    int			i;
    double		Cycle_time, Discriminant;


    /*if ( Planner_number )
        return 1;*/

    if ( Nb > PLANNER_MAX_NB )
        return 2;

    planner_sampling_time = Stime;
    planner_time = 0;
    planner_number = Nb;

    /* Initialisation des trajectoires */

    for ( i = 0; i < Nb; i++ )
    {

        /* Initialisation des objectifs */

        PPlanner[i].Ini = Ini[i];
        PPlanner[i].Fin = Fin[i];

        PPlanner[i].Min = Min[i];
        PPlanner[i].Max = Max[i];

        if ( PPlanner[i].Min != PPlanner[i].Max )
        {

            /* On teste les butees */

            if ( 	( PPlanner[i].Ini < PPlanner[i].Min ) ||
                    ( PPlanner[i].Fin < PPlanner[i].Min ) ||
                    ( PPlanner[i].Ini > PPlanner[i].Max ) ||
                    ( PPlanner[i].Fin > PPlanner[i].Max ) )
            {
                rtPPlanner_Reset();
                return 3;
            }
        }

        if ( Amax == NULL )
            PPlanner[i].Amax = PLANNER_INFINITE_ACCEL;
        else
            PPlanner[i].Amax = Amax[i];

        PPlanner[i].Vmax = Vmax[i];

        if ( ( PPlanner[i].Amax <= 0 ) || ( PPlanner[i].Vmax <= 0 ) )
        {
            rtPPlanner_Reset( );
            return 4;
        }

        /* Calcul des parametres */

        /* Variation angulaire et sens de deplacement */

        PPlanner[i].Delta = fabs( PPlanner[i].Fin - PPlanner[i].Ini );
        PPlanner[i].Direction = sgn( PPlanner[i].Fin - PPlanner[i].Ini );

        /* Type de trajectoire */

        if ( sqrt( PPlanner[i].Amax * PPlanner[i].Delta ) < PPlanner[i].Vmax )
            PPlanner[i].Type = 0;
        else
            PPlanner[i].Type = 1;

        /* Temps de cycle */

        if ( !PPlanner[i].Type )
            PPlanner[i].Tc = 2 * sqrt( PPlanner[i].Amax * PPlanner[i].Delta ) /
                    PPlanner[i].Amax;
        else
            PPlanner[i].Tc = PPlanner[i].Vmax / PPlanner[i].Amax +
                    PPlanner[i].Delta / PPlanner[i].Vmax;

    }

    /*
        Temps de cycle pour tous les axes =
        temps de cycle de l'axe le plus lent.
    */

    Cycle_time = 0;

    for ( i = 0; i < Nb; i++ )
        if ( PPlanner[i].Tc > Cycle_time )
            Cycle_time = PPlanner[i].Tc;

    for ( i = 0; i < Nb; i++ )
        PPlanner[i].Tc = Cycle_time;


    /*
        Ajustement de l'acceleration ou de la vitesse maximale
        en fonction du nouveau temps de cycle
    */

    for ( i = 0; i < Nb; i++ )
    {
        if ( !PPlanner[i].Type )
        {
            /* Cas d'une trajectoire /\ */
            PPlanner[i].Ac = 4 * PPlanner[i].Delta / pow2( PPlanner[i].Tc );
        }
        else
        {
            /* Cas d'une trajectoire /-\ */
            Discriminant = pow2( PPlanner[i].Tc * PPlanner[i].Amax )
                    - 4 * PPlanner[i].Delta * PPlanner[i].Amax;

            PPlanner[i].Vc = 0.5 * ( PPlanner[i].Tc * PPlanner[i].Amax -
                                     sqrt( Discriminant ) );

        }
    }

    /*
        Definition des differents instants de changement de type de
        trajectoire.
    */

    for ( i = 0; i < Nb; i++ )
    {
        if ( !PPlanner[i].Type )
            PPlanner[i].t1 = PPlanner[i].Tc  / 2;
        else
        {
            PPlanner[i].t1 = PPlanner[i].Vc / PPlanner[i].Amax;
            PPlanner[i].t2 = PPlanner[i].Tc - PPlanner[i].t1;
        }
    }

    return 0;
}


/**
 * @fn      rtPPlanner_Update
 * @brief   Mise à jour de la trajectoire avec le calcul de l'échantillon courant de la trajectoire
 * @param   Reference
 * @return  code d'erreur
 */
int rtPPlanner_Update( double *Reference )
{

    int			i, j;
    double		test;
    int			flag;

    if ( !planner_number )
        return 1;

    /* Mise a jour du temps */

    planner_time += planner_sampling_time;

    for ( i = 0; i < planner_number; i++ )
    {

        if ( !PPlanner[i].Type )
        {

            /* Cas d'une trajectoire /\ */

            if ( planner_time < PPlanner[i].t1 )
            {

                /* Phase acceleration croissante */

                Reference[i] = 	PPlanner[i].Ini +
                        PPlanner[i].Direction *
                        0.5 * PPlanner[i].Ac * pow2( planner_time );
            }
            else
            {

                /* Phase acceleration decroissante */

                Reference[i] = 	PPlanner[i].Ini +
                        PPlanner[i].Direction *
                        ( 	PPlanner[i].Delta / 2 +
                            PPlanner[i].Ac * PPlanner[i].t1 * ( planner_time - PPlanner[i].t1 ) -
                            0.5 * PPlanner[i].Ac * pow2( planner_time - PPlanner[i].t1 )
                            );

            }

        }
        else
        {

            /* Cas d'une trajectoire /-\ */

            if ( planner_time < PPlanner[i].t1 )
            {

                /* Phase acceleration croissante */

                Reference[i] = 	PPlanner[i].Ini +
                        PPlanner[i].Direction *
                        0.5 * PPlanner[i].Amax * pow2( planner_time );

            }

            if ( ( planner_time >= PPlanner[i].t1 )
                 && ( planner_time < PPlanner[i].t2 ) )
            {

                /* Phase vitesse constante */

                Reference[i] = 	PPlanner[i].Ini +
                        PPlanner[i].Direction *
                        ( 	0.5 * PPlanner[i].Amax * pow2( PPlanner[i].t1 ) +
                            PPlanner[i].Vc * ( planner_time - PPlanner[i].t1 )
                            );
            }

            if ( planner_time >= PPlanner[i].t2 )
            {

                /* Phase acceleration decroissante */

                Reference[i] = 	PPlanner[i].Ini +
                        PPlanner[i].Direction *
                        ( 	0.5 * PPlanner[i].Amax * pow2( PPlanner[i].t1 ) +
                            PPlanner[i].Vc * ( planner_time - PPlanner[i].t1 ) -
                            0.5 * PPlanner[i].Amax * pow2( planner_time - PPlanner[i].t2 )
                            );
            }

        }

    }

    /* Test si on a fini */

    if ( planner_time >= PPlanner[0].Tc )
    {

        for ( i = 0; i < planner_number; i++ )
            Reference[i] = 	PPlanner[i].Fin;

        planner_number = 0;

        return 1;
    }


    return 0;
}

/**
 * @fn      rtPPlanner_Reset
 * @brief   Remise à zéro du générateur de trajectoire
 * @return
 */
int rtPPlanner_Reset( void )
{

    planner_number = 0;

    return 0;
}
