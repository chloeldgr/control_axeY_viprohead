#include "rtPIDControl.h"


PID_CONTROL_STRUCT C_PID[NB_MAX_PID];

/**
 * @brief rtPIDControl_Init
 * @param id
 * @param new_param
 * @return
 */
int rtPIDControl_Init(int id,
                      double new_param[NB_PARAM])
{
    int i;
    C_PID[id].ControlType  = new_param[CONTR_TYPE];
    /// Affectation des paramÃ¨tres du correcteur
    C_PID[id].Kp       = new_param[KP];
    C_PID[id].Ki       = new_param[KI];
    C_PID[id].Kd       = new_param[KD];
    C_PID[id].Tf       = new_param[TF];
    C_PID[id].Te       = new_param[TE];
    C_PID[id].max_sat  = new_param[SAT_MAX];
    /// Initialisation des variables contenant les signaux
    for ( i = 0 ; i < NB_SAMPLES ; i++)
    {
        C_PID[id].Measure[i]    = 0.0;
        C_PID[id].Reference[i]  = 0.0;
        C_PID[id].Control[i]    = 0.0;
        C_PID[id].Error[i]      = 0.0;
        C_PID[id].U_P[i]        = 0.0;
        C_PID[id].U_I[i]        = 0.0;
        C_PID[id].U_D[i]        = 0.0;
    }

    return 0;
}


/**
 * @fn      rtPIDControl_Update()
 * @brief Mise à jour de la commande en fonction de la position et de la mesure
 *          En continue la forme du correcteur est la suivante:
 *          C(s) = Kp + Ki/s + Kd*s/(Tf*s+1)
 *          Le forme du controleur PID est la suivante e(k) = r(k) - y(k) (reference - mesure)
 *          C(z) = Kp + Ki*Te/(z-1) +Kd*(1/(Tf+Te/(z-1))) (pidtuner Matlab)
 *
 *          u(k) = u_P(k)+u_i(k)+u_d(k)
 * @param id
 * @param Measure :  mesue courante
 * @param Reference : consigne
 * @return Commande
 */
int rtPIDControl_Update(    int id,
                            double Measure,
                            double Reference,
                            double *control)
{
    int i;
    double buf_control;
    /* DÃ©calage des mesures*/
    for (i = NB_SAMPLES; i >0 ;i--)
    {
        C_PID[id].Error[i]      = C_PID[id].Error[i-1];
        C_PID[id].Control[i]    = C_PID[id].Control[i-1];
        C_PID[id].U_D[i]        = C_PID[id].U_D[i-1];
        C_PID[id].U_I[i]        = C_PID[id].U_I[i-1];
    }
    C_PID[id].Measure[0]    = Measure;
    C_PID[id].Reference[0]  = Reference;
    C_PID[id].Error[0]      = C_PID[id].Reference[0]-C_PID[id].Measure[0];

    // Calcul de l'action proportionnelle
    C_PID[id].U_P[0]        = C_PID[id].Kp*C_PID[id].Error[0];

    // Calcul de l'action intégrale
    if ( C_PID[id].Ki > 0)
        C_PID[id].U_I[0]        = C_PID[id].U_I[1] + C_PID[id].Te*C_PID[id].Ki*C_PID[id].Error[1];
    else
        C_PID[id].U_I[0]        = 0;


    // Calcul de l'action dérivée
    if ( C_PID[id].Kd > 0)
    {
        C_PID[id].U_D[0]        =   (C_PID[id].Tf-C_PID[id].Te)/C_PID[id].Tf*C_PID[id].U_D[1];
        C_PID[id].U_D[0]        +=  C_PID[id].Kd/C_PID[id].Tf*(C_PID[id].Error[0]-C_PID[id].Error[1]);
    }
    else
        C_PID[id].U_D[0]        =   0;


    // Calcul de la commande
    buf_control             = C_PID[id].U_P[0] + C_PID[id].U_I[0] + C_PID[id].U_D[0];

    // antisaturation du terme intergrale
    if ( C_PID[id].Ki > 0)
    {
        if ( buf_control > C_PID[id].max_sat)
            C_PID[id].U_I[0] = C_PID[id].max_sat -  (C_PID[id].Kp*C_PID[id].Error[0]+C_PID[id].U_D[0]);
        else
        {
            if ( buf_control < -C_PID[id].max_sat)
                C_PID[id].U_I[0] = -C_PID[id].max_sat -  (C_PID[id].Kp*C_PID[id].Error[0]+C_PID[id].U_D[0]);
            else
                C_PID[id].U_I[0] = C_PID[id].U_I[1] + C_PID[id].Te*C_PID[id].Ki*C_PID[id].Error[1];
        }
    }

    C_PID[id].Control[0]  = C_PID[id].U_P[0] + C_PID[id].U_I[0] + C_PID[id].U_D[0];
    *control =C_PID[id].Control[0];
    return  0;

}
