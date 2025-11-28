#ifndef __RT_PIDCONTROL_H__
#define __RT_PIDCONTROL_H__

#define NB_SAMPLES  2
#define NB_MAX_PID  10
#define NB_PARAM    7
// Identifiants des paramètres du correcteur
#define KP          0
#define KI          1
#define KD          2
#define TF          3
#define TE          4
#define CONTR_TYPE  5
#define SAT_MAX     6

typedef struct{
    // paramaÃ¨tres du correcteur PID
    double Kp;  // Gain du correcteur P
    double Ki;  // gain du correcteur I
    double Kd;  // Gain du correcteur D
    double Tf;  // Gain du prÃ©filtre sur le correceur D
    double Te;  // PÃ©riode d'Ã©chantillonnage
    double max_sat;
    double Measure[NB_SAMPLES];
    double Reference[NB_SAMPLES];
    double Error[NB_SAMPLES];
    double Control[NB_SAMPLES];
    double U_P[NB_SAMPLES];
    double U_I[NB_SAMPLES];
    double U_D[NB_SAMPLES];
    int ControlType;// Forme du correcteur PID ParallÃ¨le = 1

}PID_CONTROL_STRUCT;


extern int rtPIDControl_Init(       int id,
                                    double new_param[NB_PARAM]);
extern int rtPIDControl_Update(     int id,
                                    double Measure,
                                    double Reference,
                                    double *Control);


#endif
