#ifndef __EC_CONTROLLER_H_
#define __EC_CONTROLLER_H_
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../CommonLibs/rtBeckhoffModules/ec_el1008.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el3164.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el4104.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el3104.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el4132.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el5101.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el2212.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el2828.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el2004.h"
#include "../CommonLibs/rtBeckhoffModules/ec_el2124.h"
#include "../CommonLibs/rtControllerModules/rtAuxeticInchwormActuator.h"

#include "../CommonLibs/rtControllerModules/rtController_Common_Parameters.h"

#define __DEBUG__ 1
#define __DEBUG_1 0

#define PFX "rtSlaveController : "
#define DATALOG_FILENAME "/root/develop/rtController/robot_data.log"


/* Définition des PIPES de communication avec l'IHM*/
#define ID_PIPE_CMD         15
#define ID_PIPE_ACKNO       16
#define ID_PIPE_STATUS      17
#define ID_PIPE_SAVE_DATA   18
#define ID_PIPE_PARAM       19
#define ID_PIPE_IDENT_PARAM 20



//#define BusCouplerPos         0, 0    //EK1100
#define AnaOut1SlavePos       0, 0    //EL4104
#define AnaIn1SlavePos        0, 1    //EL3164
#define DigIn1SlavePos        0, 2    //EL1008
#define AnaIn2SlavePos        0, 3    //EL3164
#define AnaIn3SlavePos        0, 4    //EL3164
#define AnaIn4SlavePos        0, 5    //EL3104
#define AnaIn5SlavePos        0, 6    //EL3104
#define AnaOut2SlavePos       0, 7    //EL4132
#define Counter1SlavePos      0, 8    //EL5101
#define DigOut1SlavePos       0, 9    //EL2004
#define DigIn2SlavePos        0, 10    //EL1008
#define DigIn3SlavePos        0, 11    //EL1008
#define DigOut2SlavePos       0, 12    //EL2828
#define ECExtenderSlavePos       0, 13    //EL1110
//#define DigOut1SlavePos       0, 10    //EL2212
//#define DigOut2SlavePos       0, 11    //EL2212


#define NB_EL5101   1
#define NB_EL3164   3
#define NB_EL3104   2
#define NB_EL4104   1
#define NB_EL4132   1
#define NB_EL1008   1
#define NB_EL2212   2
#define NB_EL2004   1
#define NB_EL2828   1
//#define NB_EL2124   1



#define Beckhoff_EK1100     0x00000002, 0x05dd2c52
#define Beckhoff_EK1110     0x00000002, 0x04562c52

/*** Ethercat Master State ***/
#define EC_MASTER_INIT      0x00
#define EC_MASTER_PREOP     0x02
#define EC_MASTER_SAFEOP    0x04
#define EC_MASTER_OP        0x0A // A changer si AL_STATE 0x08


/***   PARAMETRES DU SYSTEME ***/
#define NB_COUNTER              1


/*** ERREURS ***/
#define __NO_ERROR          0
#define NO_ERROR            0

/*** SUBMODES ****/
#define RESET_CNT_SET_CAL_SUBMODE           1
#define WRITE_CNT_VALUE_CAL_SUBMODE         2
#define SET_CNT_SET_CAL_SUBMODE             3
#define RESET_POS_CAL_SUBMODE               4
/*** SUBMODE de Calibration***/
#define WAIT_SYRINGE_POSITIONNING_SUBMODE   5
#define SEARCH_PISTON_SUBMODE               6
#define WAIT_LOCK_SYRINGE_SUBMODE           7

/// PHASES FOR INCHWORM ACTUATOR
#define INCHWORM_PHASE_S0   100
#define INCHWORM_PHASE_S1   101
#define INCHWORM_PHASE_S2   102
#define INCHWORM_PHASE_S3   103
#define INCHWORM_PHASE_S4   104
#define INCHWORM_PHASE_S5   105
#define INCHWORM_PHASE_S6   106





/**/
#define CONNECT_MASTER_DEVICE_SUBMODE               110
#define CONFIGURE_MASTER_DEVICE_SUBMODE             111
#define DISCONNECT_MASTER_DEVICE_SUBMODE            112
#define CONTROL_TELEOP_SUBMODE                      113

/* Les sous mode de l'identification*/
#define STEP_IDENTIFICATION         1
#define HARMONIC_IDENTIFICATION     2




const double sampling_period_s =0.001; // période d'échantillonnage en s

#define FREQUENCY 1000	// Hz
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)
#define NB_PARAM 10
#define NB_TX_DATA 10


#define NB_ANALOG_INPUT 20
#define NB_ANALOG_OUTPUT 6
#define NB_DIG_INPUT    2
#define NB_DIG_OUTPUT   16

// Identifiant
#define ANA_OUT_1 0 // Exc
#define ANA_OUT_2 1 //
#define ANA_OUT_3 2 // W1+
#define ANA_OUT_4 3 // W2+
#define ANA_OUT_5 4
#define ANA_OUT_6 5


#define DIG_OUT_1 0
#define DIG_OUT_2 1
#define DIG_OUT_3 2
#define DIG_OUT_4 3
#define DIG_OUT_5 4
#define DIG_OUT_6 5
#define DIG_OUT_7 6
#define DIG_OUT_8 7
#define DIG_OUT_9 8
#define DIG_OUT_10 9
#define DIG_OUT_11 10
#define DIG_OUT_12 11
#define DIG_OUT_13 12   //EV1
#define DIG_OUT_14 13   //EV2
#define DIG_OUT_15 14   //EV3
#define DIG_OUT_16 15   //EV4
#define DIG_OUT_17  16
#define DIG_OUT_18  17
#define DIG_OUT_19  18
#define DIG_OUT_20  19


typedef struct
{
    cmd_type    cmd;
    mode_type   mode;
    mode_type   submode;
    /// Flags de fonctionnements
    /// bit 0 : Emergency stop
    /// bit 1 : bIsRunning
    /// bit 2 : bIsInit
    /// bit 3 : bIsCal
    unsigned int status_word;



    double analog_input[NB_ANALOG_INPUT];
    double analog_output[NB_ANALOG_OUTPUT];
    int32_t encoder_value[NB_COUNTER];
    uint8_t digital_input[NB_DIG_INPUT];
    uint16_t digital_output;
    uint16_t digital_output_5V;
    long    dt;

    double init_inchworm_velocity;
    double inchworm_velocity;
}RTCONTROLLER_STRUCT;

#define NB_ACTUATORS 1
typedef struct{
    double Control[NB_ACTUATORS];
}ROBOT_STRUCT;

typedef struct{
    int16_t reg_dig_output;
    //int16_t reg_dig_output_5V;
    double analog_output[NB_ANALOG_OUTPUT];
    double analog_input[NB_ANALOG_INPUT];
    double Control[NB_ACTUATORS];
    int Inchworm_DC;
    int Inchworm_Direction;
    double Inchworm_period_ms;
    double Inchworm_actuation_period_ms;
    double Inchworm_chuck_time_ms;
}RTCONTROLLER_PARAM_STRUCT;


typedef struct{
    unsigned int phase;
    unsigned int phase_time[7];
    int direction;
}INCHWORM_STRUCT;

#define MAX_BYTES_FIFO 5000
typedef struct{
    unsigned long long current_time;    
    double analog_input[8];
    uint16_t digital_output;

}SAVE_DATA_STRUCT;



void rtController_stop(int sig);
#endif
