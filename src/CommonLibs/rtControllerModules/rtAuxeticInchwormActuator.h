#ifndef RTAUXETICINCHWORMACTUATOR_H
#define RTAUXETICINCHWORMACTUATOR_H
#define INCHWORM_PHASE_INIT             0
#define INCHWORM_PHASE_OPEN_FG          1
#define INCHWORM_PHASE_AA_ELONGATION    2
#define INCHWORM_PHASE_CLOSE_FG         3
#define INCHWORM_PHASE_OPEN_MG          4
#define INCHWORM_PHASE_AA_RETRACTION    5
#define INCHWORM_PHASE_CLOSE_MG         6

typedef struct{
    int idSolenoid; // identifiant de l'électrovanne associée
    unsigned int bStatus; // Etat du préhenseur
    unsigned long long close_time;
    unsigned long long open_time;
    unsigned long long chamber_pressure;
}GRIPPER_STRUCT;

typedef struct{
    int idSolenoid;
    unsigned long long actuation_time;
    unsigned long long chamber_pressure;
}LINEAR_AUXETIC_ACTUATOR;




typedef struct{
    unsigned int sampling_period;
    unsigned int actuation_time;
    unsigned int digital_output;
    unsigned int gripper_time;
    unsigned long long current_time;
    int direction;
    double control;
    unsigned long long init_phase_time;       // en ms
    unsigned long long phase_time[7];   //en ms
    int phase;
    unsigned long long period; /*période du cycle inchworm en ms*/
}AUXETIC_INCHWORM_ACTUATOR_STRUCT;

unsigned int rtAuxeticActuator_initParam(unsigned int sampling_period,
                                         unsigned long long init_phase_time,
                                         unsigned long long sequence_period,
                                         unsigned long long gripper_period);
unsigned int rtAuxeticActuator_updateParam( int direction,
                                            unsigned long long sequence_period,
                                            unsigned long long gripper_period);
#endif
