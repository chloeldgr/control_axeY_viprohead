//Includes
#include "rtController.h"
#include "ecrt.h"
#include <math.h>
#include <stdio.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/timer.h>
#include <native/pipe.h>
#include <native/queue.h>

/* Descripteur de fichier pour le data log*/
FILE *fp;

// Définition de controller
RTCONTROLLER_STRUCT controller={    CMD_NO_CMD,            // cmd
                                    WAIT_MODE,             // mode
                                    WAIT_SUBMODE,          // submode
                                    0,
                                    {}
                               };
SAVE_DATA_STRUCT save_data;

/// Paramètres spécifique au système
RTCONTROLLER_PARAM_STRUCT        controller_parameters;
INCHWORM_STRUCT                     inchworm_actuator = {INCHWORM_PHASE_S0,
                                                         {0,0,0,0,0,0,0},
                                                        1};
ROBOT_STRUCT robot={{0}};
AUXETIC_INCHWORM_ACTUATOR_STRUCT auxetic_actuator;

// EtherCAT
static ec_master_t          *master        = NULL;
static ec_master_state_t    master_state   = {};
static ec_domain_t          *domain        = NULL;
static ec_domain_state_t    domain_state   = {};
static uint8_t              *domain_pd     = NULL;

//Xenomai Thread Fifo
#define TASK_PRIO 0                 /* Highest RT priority */
#define TASK_MODE T_FPU|T_CPU(0)    /* Uses FPU, bound to CPU #0 */
#define TASK_STKSZ 4096             /* Stack size (in bytes) */

RT_TASK controller_main_task;
RT_TASK connect_device_task;
RT_TASK rtController_SlaveTask;


RT_PIPE pipe_cmd;
RT_PIPE pipe_ackno;
RT_PIPE pipe_status;
RT_PIPE pipe_save_data;
RT_PIPE pipe_param;
RTIME current_time;
RTIME previous_time;


// Beckhoff modules
const uint8_t EL5101_Enable_C_Reset[]          = {0};

/****************************************************************************/
static uint64_t dc_start_time_ns = 0LL;
static uint64_t dc_time_ns = 0;
#define SYNC_MASTER_TO_REF 0
#if SYNC_MASTER_TO_REF
static uint8_t  dc_started = 0;
static int32_t  dc_diff_ns = 0;
static int32_t  prev_dc_diff_ns = 0;
static int64_t  dc_diff_total_ns = 0LL;
static int64_t  dc_delta_total_ns = 0LL;
static int      dc_filter_idx = 0;
static int64_t  dc_adjust_ns;
#endif
static int64_t  system_time_base = 0LL;
/*static uint64_t wakeup_time = 0LL;
static uint64_t overruns = 0LL;*/

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
    (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
/****************************************************************************/
const struct timespec cycletime = {0, 1000000};

/****************************************************************************/

/* Internal fonctions*/
u_int64_t system_time_ns(void)
{
    RTIME time = rt_timer_read();

    if (system_time_base > time)
    {
        rt_printf("%s() error: system_time_base greater than"
                  " system time (system_time_base: %lld, time: %llu\n",
                  __func__, system_time_base, time);
        return time;
    }
    else
    {
        return time - system_time_base;
    }

}

void PRINTF_RTCONTROLLER(char * text)
{
    fprintf(stdout,text);
    fflush(stdout);
}
/******/
int read_data_log_file(void)
{
    double position[4];
    int i;
    // Ouverture du fichier
    fp=fopen(DATALOG_FILENAME , "r+");
    if (fp != NULL )
    {
        //        fscanf(fp,"%lf %lf %lf %lf", &position[0], &position[1], &position[2], &position[3]);
        //        for ( i = 0 ; i < NB_POSITION_SENSORS ; i++)
        //            encoder_init[i] = (int)(position[i]/(joint_direction[i]*encoder_top_to_SI[i]));
        //        printf("%f %f %f %f\n",position[0],position[1],position[2],position[3]);
        //        printf("%d %d %d %d\n",encoder_init[0],encoder_init[1],encoder_init[2],encoder_init[3]);
    }

    fclose(fp);
    return __NO_ERROR;

}


int write_data_log_file(void)
{
    // Ouverture du fichier
    printf("Ecriture dans le fichier de log\n");
    fp=fopen(DATALOG_FILENAME , "w+");
    if (fp != NULL )
    {

        //        fprintf(fp,"%f %f %f %f\n", controller.JPos[0],
        //                                    controller.JPos[1],
        //                                    controller.JPos[2],
        //                                    controller.JPos[3]);
    }
    else
        printf("Echec ouverture fichier log\n");

    fclose(fp);
    return __NO_ERROR;
}

/******/
/**********************************************************/
/******* FONCTIONS BUS ETHERCAT ***************************/
/**********************************************************/
/**
  * @fn     void rt_check_domain_state(void
  * @brief
  * @arg    aucun
  * @return aucun
 */
void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter) {
        rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain_state.wc_state) {
        rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain_state = ds;
}

//Fonction de check de l'etat du master******************************************
/**
  * @fn     void rt_check_master_state()
  * @brief
  * @arg    aucun
  * @return aucun
 */
void rt_check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        fprintf(stdout,"AL states: 0x%02X.\n", ms.al_states);
        fflush(stdout);// rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up)
    {

        rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;

}

/**********************************************************/
/***                                                    ***/
/***     FONCTIONS ACQUISITION/CONTROLE                 ***/
/***                                                    ***/
/**********************************************************/


/**
/** * @fn       rtReadAnalogInput(void)
/** * @brief    Lecture de l'ensemble des entrées analogiques et affectations des valeurs
/** * @return   Code d'erreur
/** */
int rtReadAnalogInput(void)
{
    int i;


    if (EL3164_Read(0, domain_pd) < 0)
        return ERROR_READ_ANALOG_INPUT;

    EL3164_GetValue(0, 0, &controller.analog_input[0]);// VPPM1 capteur pression
    EL3164_GetValue(0, 1, &controller.analog_input[1]);
    EL3164_GetValue(0, 2, &controller.analog_input[2]);// VPPM2 capteur pression
    EL3164_GetValue(0, 3, &controller.analog_input[3]);


    if (EL3164_Read(1, domain_pd) < 0)
        return ERROR_READ_ANALOG_INPUT;
    EL3164_GetValue(1, 0, &controller.analog_input[4]); //Capteur S1
    EL3164_GetValue(1, 1, &controller.analog_input[5]); //Capteur S2
    EL3164_GetValue(1, 2, &controller.analog_input[6]); //Capteur S3
    EL3164_GetValue(1, 3, &controller.analog_input[7]); //Capteur S4


    if (EL3164_Read(2, domain_pd) < 0)
        return ERROR_READ_ANALOG_INPUT;
    EL3164_GetValue(2, 0, &controller.analog_input[8]); //NC
    EL3164_GetValue(2, 1, &controller.analog_input[9]); //NC
    EL3164_GetValue(2, 2, &controller.analog_input[10]); //NC
    EL3164_GetValue(2, 3, &controller.analog_input[11]); //NC

    if (EL3104_Read(0, domain_pd) < 0)
        return ERROR_READ_ANALOG_INPUT;
    EL3104_GetValue(0, 0, &controller.analog_input[12]); //ANA_IN_13
    EL3104_GetValue(0, 1, &controller.analog_input[13]); // NC
    EL3104_GetValue(0, 2, &controller.analog_input[14]); //A2
    EL3104_GetValue(0, 3, &controller.analog_input[15]); //NC
//    for (i= 0; i < NB_CHANNEL_EL3104;i++)
//        EL3104_GetValue(0, i, &controller.analog_input[i+3*NB_CHANNEL_EL3164]); //12 13 14 15

    if (EL3104_Read(1, domain_pd) < 0)
        return ERROR_READ_ANALOG_INPUT;
        EL3104_GetValue(1,0, &controller.analog_input[16]);//NC
        EL3104_GetValue(1,1, &controller.analog_input[17]);//NC
        EL3104_GetValue(1,2, &controller.analog_input[18]);//NC
        EL3104_GetValue(1,3, &controller.analog_input[19]);//NC
//        for (i= 0; i < NB_CHANNEL_EL3164;i++)
//        EL3104_GetValue(1, i, &controller.analog_input[i+3*NB_CHANNEL_EL3164+NB_CHANNEL_EL3104]);//16 17 18 19

    return NO_ERROR;
}

/**
  * @fn rtRead_Encoder
 * @brief lecture des codeurs
 * @return code d'erreur
 */
int rtReadEncoder(void)
{
    int i;
    // Lecture des compteurs
    for ( i = 0; i < NB_COUNTER; i++)
        EL5101_Read(i,  &controller.encoder_value[i]);
    return NO_ERROR;
}


/**
 * @brief rtController_SetS1
 * @param status
 */
void rtController_SetS1(int status)
{

    if ( status == 1)
        controller.digital_output |=(1 << 2);
    else
        controller.digital_output &=~(1 << 2);

}

/**
 * @brief rtController_SetS2
 * @param status
 */
void rtController_SetS2(int status)
{
    if ( status == 1)
        controller.digital_output |=(1 << 1);
    else
        controller.digital_output &=~(1 << 1);

}

/**
 * @brief rtController_SetS3
 * @param status
 */
void rtController_SetS3(int status)
{
    if ( status == 1)
    {
        controller.digital_output |=(1 << 0);
        controller.digital_output |=(1 << 3);
    }
    else
    {
        controller.digital_output &=~(1 << 0);
        controller.digital_output &=~(1 << 3);
    }

}

/**
 * @brief rtController_SetS4
 * @param status
 */
void rtController_SetS4(int status)
{
    if ( status == 1)
    {
        controller.digital_output &=~(1 << 0);
        controller.digital_output |=(1 << 3);
    }
    else
    {
        controller.digital_output &=~(1 << 0);
        controller.digital_output &=~(1 << 3);
    }

}


/**
  * @fn rtReadDigInput
 * @brief Lecture des entrées numériques puis mise interprétation
 * @return
 */
int rtReadDigInput(void)
{
    uint8_t reg;

    if (EL1008_Read(0, domain_pd, &controller.digital_input[0]) < 0)
        return ERROR_READ_DIG_INPUT;
    if (EL1008_Read(1, domain_pd, &controller.digital_input[1]) < 0)
        return ERROR_READ_DIG_INPUT;

    return __NO_ERROR;
}

/**
  * @fn rtWriteDigOutput()
 * @brief Ecriture sortie numérique : construction de la trame en fonction du hardware
 * @return
 */
int rtWriteDigOutput(void)
{

    if (EL2004_Write(0,domain_pd,controller.digital_output)<0)
        return ERROR_WRITE_DIG_OUTPUT;

    return __NO_ERROR;
}


/**
  * @fn rtWriteAnalogValue()
 * @brief Ecriture d'une tension analogique sur les CNA
 * @return
 */
int rtWriteAnalogValue(void)
{
    double buf_el4104[NB_EL4104_CHANNEL];
    double buf_el4132[NB_EL4132_CHANNEL];
    buf_el4104[0]= controller.analog_output[0];
    buf_el4104[1]= controller.analog_output[1];
    buf_el4104[2]= controller.analog_output[2];
    buf_el4104[3]= controller.analog_output[3];
    buf_el4132[0]= controller.analog_output[4];
    buf_el4132[1]= controller.analog_output[5];

    if ( EL4104_Write(0, domain_pd, buf_el4104))
        return ERROR_WRITE_ANALOG_OUTPUT;

    if ( EL4132_Write(0, domain_pd, buf_el4132))
        return ERROR_WRITE_ANALOG_OUTPUT;

    return NO_ERROR;
}


/**********************************************************/
/******* FONCTIONS COMMUNICATION AVEC  IHM ****************/
/**********************************************************/
/**
  * \fn     void rtOpenFIFO(void)
  * \arg    aucun
  * \return code d'erreur
  * \brief  Création et ouverture des FIFOs pour la communication avec l'IHM
  */
int rtOpenFIFO(void)
{
    int ret;


    /* FIFO CMD*/
    ret = rt_pipe_create(&pipe_cmd,
                         "rtp15",
                         ID_PIPE_CMD,
                         sizeof(cmd_type));
    if(ret<0)
    {
        rt_printf(PFX "Impossible d'ouvrir la pipe de commande %d\n",ret);
        return ERROR_FIFO_OPEN_FAILED;
    }
    /* fifo ACKNO*/
    ret =rt_pipe_create(&pipe_ackno,
                        "rtp16",
                        ID_PIPE_ACKNO,
                        sizeof(cmd_type));
    if(ret<0)
    {
        rt_printf(PFX "Impossible d'ouvrir la pipe de ackno");
        return ERROR_FIFO_OPEN_FAILED;
    }
    /* FIFO STATUS*/
    ret = rt_pipe_create(&pipe_status,
                         "rtp17",
                         ID_PIPE_STATUS,
                         sizeof(RTCONTROLLER_STRUCT));
    if(ret<0)
    {

        rt_printf(PFX "Impossible d'ouvrir la pipe de status");
        return ERROR_FIFO_OPEN_FAILED;
    }
    /* FIFO SAVE DATA*/
    ret = rt_pipe_create(&pipe_save_data,
                         "rtp18",
                         ID_PIPE_SAVE_DATA,
                         MAX_BYTES_FIFO*sizeof(SAVE_DATA_STRUCT));
    if(ret<0)
    {
        rt_printf(PFX "Impossible d'ouvrir la pipe save data\n");
        return ERROR_FIFO_OPEN_FAILED;
    }
    /* FIFO PARAM*/
    ret = rt_pipe_create(&pipe_param,
                         "rtp19",
                         ID_PIPE_PARAM,
                         sizeof(RTCONTROLLER_PARAM_STRUCT));
    if(ret<0)
    {
        rt_printf(PFX "Impossible d'ouvrir la pipe param\n");
        return ERROR_FIFO_OPEN_FAILED;
    }
    return 0;
}
/**
  * \fn void rtCloseFIFO(void)
  * \arg aucun
  * \return aucun
  * \brief Fermeture des FIFO
  */
void rtCloseFIFO(void)
{
    rt_printf(PFX "Fermeture des FIFO\n");
    rt_pipe_delete(&pipe_cmd);
    rt_pipe_delete(&pipe_ackno);
    rt_pipe_delete(&pipe_status);
    rt_pipe_delete(&pipe_save_data);
    rt_pipe_delete(&pipe_param);
    rt_printf(PFX "FIFO fermees\n");
}



/**
  * @fn rtSendDataToSave
 * @brief procédure permettant la sauvegarde des données
 * @return
 */
int rtSendDataToSave(void)
{
    int ret, i ;
    save_data.current_time = rt_timer_read();

    /* remplissage des paramètres de la structure données pour le sauvegarde de données*/
    /*for (i = 0; i < NB_TX_DATA ;i ++)*/
    save_data.analog_input[0] = controller.analog_input[4]; // Capteur pression 1
    save_data.analog_input[1] = controller.analog_input[5];
    save_data.analog_input[2] = controller.analog_input[6];
    save_data.analog_input[3] = controller.analog_input[7];
    save_data.analog_input[4] = controller.analog_input[12];
    save_data.analog_input[5] = controller.analog_input[13];
    save_data.analog_input[6] = controller.analog_input[14];
    save_data.analog_input[7] = controller.analog_input[15];


    save_data.digital_output  = controller.digital_output;
    /* envoi de la structure dans la fifo*/
    ret = rt_pipe_stream(	&pipe_save_data,
                            &save_data,
                            sizeof ( SAVE_DATA_STRUCT ));
    if ( ret < 0 )
    {
        rt_printf(PFX"Echec envoi des données vers l'interface user\n");
        return ret;
    }
    return __NO_ERROR;

}

unsigned int rtController_GetCalStatus(void)
{
    return (controller.status_word & (1u <<3));
}
void rtController_SetCalStatus(unsigned int value)
{
    switch (value) {
    case 0:

        controller.status_word = controller.status_word & ~(1<<3);
        break;
    case 1:
        controller.status_word = controller.status_word |(1<<3);
        break;

    default:
        break;
    }
}


unsigned int rtController_GetInitStatus(void)
{
    return (controller.status_word & (1u <<2));
}
void rtController_SetInitStatus(unsigned int value)
{
    switch (value) {
    case 0:

        controller.status_word = controller.status_word & ~(1<<2);
        break;
    case 1:
        controller.status_word = controller.status_word |(1<<2);
        break;

    default:
        break;
    }
}
unsigned int rtController_GetRunningStatus(void)
{
    return (controller.status_word & (1u <<1));
}
void rtController_SetRunningStatus(unsigned int value)
{
    switch (value) {
    case 0:

        controller.status_word = controller.status_word & ~(1<<1);
        break;
    case 1:
        controller.status_word = controller.status_word |(1<<1);
        break;

    default:
        break;
    }
}


/**
  * @fn     void rtProcessCommand();
  * @brief
  * @arg    aucun
 * @return  aucun
 */
void rtProcessCommand(void)
{
    int ret, i;
    cmd_type new_cmd;
    RTCONTROLLER_PARAM_STRUCT __local_param;

    /*Gestion de la communication par les pipes avec l'interface */
    controller.cmd = CMD_NO_CMD;

    /** Lecture de la commande dans la fifo cmd**/
    ret = rt_pipe_read( &pipe_cmd,
                        &new_cmd ,
                        sizeof(cmd_type),
                        TM_NONBLOCK);
    if ( ret > 0 )
        controller.cmd = new_cmd;
    /** Interprétation de la commande**/
    switch(controller.cmd)
    {
    /** Pa s de nouvelle requête **/
    case CMD_NO_CMD:

        //Pas d'action

        break;

        /// requête de récupération de l'état courant du système
    case CMD_GET_STATUS:
#if __DEBUG_1 == 1
        rt_printf(PFX "Reception de la commande Get Status\n");
#endif
        // Mise Ã  disposition de l'Ã©tat courant du controleur dans la fifo_status
        if ( rt_pipe_write(&pipe_status,
                           &controller,
                           sizeof(RTCONTROLLER_STRUCT),
                           P_NORMAL)<0)
        {
            rt_printf(PFX" Echec ecriture dans la fifo status\n");
            break;
        }

        break;
    case CMD_DO_CAL:
        rt_printf(PFX "Reception de la commande Do Cal\n");
        /// requête pour procéder à l'étalonnage du système
        /// Si le robot est déjà étalonné on passe
        if ( rtController_GetCalStatus())
        {
            rt_printf(PFX" Robot déjà étalonné\n");
            break;
        }

        controller.mode     = CAL_MODE;
        if ( controller.submode == WAIT_SUBMODE )
        {
            controller.submode = RESET_CNT_SET_CAL_SUBMODE;
            rt_printf(PFX "RESET_CNT_SET_CAL_SUBMODE\n");
        }

        else
        {
            rtController_SetCalStatus(1);
        }

        break;
    case CMD_WRITE_DIG_OUT:
        // lecture de la FIFO PARAM
        if ( rt_pipe_read(&pipe_param,
                          &controller_parameters,
                          sizeof(RTCONTROLLER_PARAM_STRUCT),
                          TM_NONBLOCK) < 0)
            break; // Gestion de l'erreur
        controller.digital_output = controller_parameters.reg_dig_output;

        break;
//    case CMD_WRITE_DIG_OUT_5V:
//        // lecture de la FIFO PARAM
//        if ( rt_pipe_read(&pipe_param,
//                          &controller_parameters,
//                          sizeof(RTCONTROLLER_PrtController_main_proc,ARAM_STRUCT),
//                          TM_NONBLOCK) < 0)
//            break; // Gestion de l'erreur
//        controller.digital_output_5V = controller_parameters.reg_dig_output_5V;

//        break;

    case CMD_WRITE_ANA_OUT:
        // lecture de la FIFO PARAM
        if ( rt_pipe_read(&pipe_param,
                          &controller_parameters,
                          sizeof(RTCONTROLLER_PARAM_STRUCT),
                          TM_NONBLOCK) < 0)
            break; // Gestion de l'erreur

        for (i=0; i < NB_ANALOG_OUTPUT;i++)
            controller.analog_output[i] = controller_parameters.analog_output[i];

        break;
    case CMD_EXIT:
        rtController_SetRunningStatus(0);
        write_data_log_file();
        break;
    case CMD_INIT_INCHWORM:
        //Lecture des données dans la structure de paramètres pour controler le inchworm
        if ( rt_pipe_read(&pipe_param,
                          &controller_parameters,
                          sizeof(RTCONTROLLER_PARAM_STRUCT),
                          TM_NONBLOCK) < 0)
            break; // Gestion de l'erreur
        controller.analog_output[ANA_OUT_1]=controller_parameters.analog_output[ANA_OUT_1];
        controller.analog_output[ANA_OUT_2]=controller_parameters.analog_output[ANA_OUT_2];

        // bascule dans la mode CTRL INCHOWMR
        previous_time = rt_timer_read();
        controller.dt = 0;
        controller.mode     = CTRL_INCHWORM_MODE;
        auxetic_actuator.period = controller_parameters.Inchworm_period_ms;
        auxetic_actuator.gripper_time = controller_parameters.Inchworm_chuck_time_ms;
        auxetic_actuator.direction = controller_parameters.Inchworm_Direction;

        controller.digital_output = rtAuxeticActuator_initParam(1, // période d'échantillonnage
                                                                250, // phase d'initialisation
                                                                auxetic_actuator.period,
                                                                auxetic_actuator.gripper_time);


        break;

    case CMD_CTRL_INCHWORM:
        //Lecture des données dans la structure de paramètres pour controler le inchworm
        if ( rt_pipe_read(&pipe_param,
                          &controller_parameters,
                          sizeof(RTCONTROLLER_PARAM_STRUCT),
                          TM_NONBLOCK) < 0)
            break; // Gestion de l'erreur
        controller.analog_output[ANA_OUT_1]=controller_parameters.analog_output[ANA_OUT_1];
        controller.analog_output[ANA_OUT_2]=controller_parameters.analog_output[ANA_OUT_2];

        // bascule dans la mode CTRL INCHOWMR
        previous_time = rt_timer_read();
        controller.dt = 0;
        controller.mode     = CTRL_INCHWORM_MODE;
        auxetic_actuator.period = controller_parameters.Inchworm_period_ms;
        auxetic_actuator.gripper_time = controller_parameters.Inchworm_chuck_time_ms;
        auxetic_actuator.direction = controller_parameters.Inchworm_Direction;




        break;
    case CMD_STOP_INCHWORM:
        controller.analog_output[ANA_OUT_1]=0;
        controller.analog_output[ANA_OUT_2]=0;
        controller.digital_output = 0;
        // bascule dans la mode CTRL INCHOWMR
        controller.mode = DIRECT_CTRL_MODE;

        break;


    case CMD_CTRL_FLAT_INCHWORM:
        //Lecture des données dans la structure de paramètres pour controler le inchworm
        if ( rt_pipe_read(&pipe_param,
                          &controller_parameters,
                          sizeof(RTCONTROLLER_PARAM_STRUCT),
                          TM_NONBLOCK) < 0)
            break; // Gestion de l'erreur
        // bascule dans la mode CTRL INCHOWMR
        previous_time = rt_timer_read();
        controller.dt = 0;

        // Code en dur les temps des différentes phases
        inchworm_actuator.phase_time[0] = 100;
        inchworm_actuator.phase_time[1] = controller_parameters.Inchworm_DC*controller_parameters.Inchworm_period_ms/100;
        inchworm_actuator.phase_time[2] = (100-controller_parameters.Inchworm_DC)*controller_parameters.Inchworm_period_ms/100;

        controller.mode     = CTRL_FLAT_INCHWORM_MODE;
        inchworm_actuator.phase = INCHWORM_PHASE_S1;
//        rtController_ToggleED4();
//        if ( controller_parameters.Inchworm_Direction == 1)
//            rtController_ToggleED1();


        break;
// Démarrage du mode téléopération
    case CMD_START_TELEOP:
        //Lecture des données dans la structure de paramètres pour controler le inchworm
        if ( rt_pipe_read(&pipe_param,
                          &controller_parameters,
                          sizeof(RTCONTROLLER_PARAM_STRUCT),
                          TM_NONBLOCK) < 0)
            break; // Gestion de l'erreur
        controller.analog_output[ANA_OUT_1]=controller_parameters.analog_output[ANA_OUT_1];
        controller.analog_output[ANA_OUT_2]=controller_parameters.analog_output[ANA_OUT_2];

        // bascule dans la mode CTRL INCHOWMR
        previous_time = rt_timer_read();
        controller.dt = 0;
        controller.mode     = TELEOP_MODE;
        controller.submode  = WAIT_TELEOP_SUBMODE;
        auxetic_actuator.period = controller_parameters.Inchworm_period_ms;
        auxetic_actuator.gripper_time = controller_parameters.Inchworm_chuck_time_ms;
        auxetic_actuator.direction = controller_parameters.Inchworm_Direction;

        controller.digital_output = rtAuxeticActuator_initParam(1, // période d'échantillonnage
                                                                250, // phase d'initialisation
                                                                auxetic_actuator.period,
                                                                auxetic_actuator.gripper_time);
        break;
        // Arret du mode téléopération
    case CMD_STOP_TELEOP:
        controller.analog_output[ANA_OUT_1]=0;
        controller.analog_output[ANA_OUT_2]=0;
        controller.digital_output = 0;
        // bascule dans la mode CTRL INCHOWMR
        controller.mode = DIRECT_CTRL_MODE;
        break;
    default:
        rt_printf(PFX " Error: should never occur\n");

        break;
    }
    /* Envoi de l'accuse de reception */
    if ( controller.cmd != CMD_NO_CMD )
    {
        if ( rt_pipe_write (    &pipe_ackno,
                                &controller.cmd,
                                sizeof ( cmd_type ),
                                P_NORMAL ) < 0 )
            rt_printf("Erreur Ack ");
    }



}





struct timeval tv={0,0};
static unsigned int sync_ref_counter = 0;
/**
 * @brief rtController_updateController
 * @return
 */
int rtController_updateController(void)
{
    ecrt_master_receive(master);
    ecrt_domain_process(domain);
    rt_check_master_state();


    if ( master_state.al_states == EC_MASTER_OP)
    {
        /* Mise à jour des données capteurs*/
        rtReadAnalogInput();
        rtReadEncoder();
        rtReadDigInput();
        rtWriteDigOutput();
        rtWriteAnalogValue();
        fprintf(stdout,"Analog output %f\n", controller.analog_output[0]);
        fflush(stdout);// rt_printf("AL states: 0x%02X.\n", ms.al_states);


    }
    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}

/**
  * @fn rtController_Wait_Proc()
  * @brief Procédure permettant de réaliser l'identification
  * @arg aucun
  * @return code d'erreur
  */
int rtController_Wait_Proc(void)
{

    return __NO_ERROR;
}

/**
  * @fn rtController_DoDirectCtrl_Proc()
  * @brief
  * @arg aucun
  * @return code d'erreur
  */
int rtController_DoDirectCtrl_Proc(void)
{
    return __NO_ERROR;
}

///

void rtController_DoCtrlFlatInchworm_Proc(void)
{

}


///
/// \brief rtController_DoTeleop_Proc
///
void rtController_DoTeleop_Proc(void)
{

    switch( controller.submode)
    {
    case WAIT_TELEOP_SUBMODE:

            controller.dt = 0;

            controller.analog_output[ANA_OUT_1]=controller_parameters.analog_output[ANA_OUT_1];
            controller.analog_output[ANA_OUT_2]=controller_parameters.analog_output[ANA_OUT_2];

            // bascule dans la mode CTRL INCHOWMR
            controller.dt = 0;
            controller.mode     = TELEOP_MODE;
            controller.submode  = INCHWORM_CTRL_TELEOP_SUBMODE;
            auxetic_actuator.period = controller_parameters.Inchworm_period_ms;
            auxetic_actuator.gripper_time = controller_parameters.Inchworm_chuck_time_ms;
            auxetic_actuator.direction = controller_parameters.Inchworm_Direction;

            controller.digital_output = rtAuxeticActuator_initParam(1, // période d'échantillonnage
                                                                    250, // phase d'initialisation
                                                                    auxetic_actuator.period,
                                                                    auxetic_actuator.gripper_time);

            controller.init_inchworm_velocity = controller.analog_input[13];
            controller.inchworm_velocity = controller.analog_input[13]-controller.init_inchworm_velocity;

        break;
    case INIT_TELEOP_SUBMODE:
        break;
    case INCHWORM_CTRL_TELEOP_SUBMODE:
       //
       if( controller.analog_input[12]>3)
        {
            controller.inchworm_velocity = controller.analog_input[13]-controller.init_inchworm_velocity;
            auxetic_actuator.direction = 0;
            if ( controller.inchworm_velocity > 0.1)
                auxetic_actuator.direction = 1;

            if ( controller.inchworm_velocity < -0.1)
                auxetic_actuator.direction = -1;

            auxetic_actuator.period = 2155-fabs(1555*(controller.inchworm_velocity));

            controller.digital_output = rtAuxeticActuator_updateParam(  auxetic_actuator.direction,
                                                                        auxetic_actuator.period,
                                                                        auxetic_actuator.gripper_time);
        }
        break;
    case ORIENTATION_CTRL_TELEOP_SUBMODE:
        break;
    case STOP_TELEOP_SUBMODE:
        break;
    default:
        break;
    }

}

///
/// \brief rtController_DoCtrlInchworm_Proc
///
void rtController_DoCtrlInchworm_Proc(void)
{
    int i;
    // Déclenchement des électrovannes en fonction de la fréquence et du rapport cyclique
    controller.dt++;

   //rt_printf("EC_CONTROLLER : dig input %d\n",controller.digital_input[0] & (1u << 5));
  // if ( controller.digital_input[1]&(1u<<3))
        controller.digital_output = rtAuxeticActuator_updateParam(     auxetic_actuator.direction,
                                                                        auxetic_actuator.period,
                                                                        auxetic_actuator.gripper_time);

   /* else
        controller.digital_output = 0;*/
//       controller.digital_output = rtAuxeticActuator_ControlCycle();

}

/**************************************************************************************/

/**
 * @brief Fonction principale associée au thread principale
 * @param arg
 */
void rtController_main_proc(void *arg)
{
    int ret,i;
    static unsigned int sync_ref_counter = 0;
    struct timeval tv={0,0};
    int cycle_counter = 0;

    /* PÃ©riodiser la tÃ¢che principale*/
    rt_task_set_periodic(NULL,
                         TM_NOW,
                         PERIOD_NS); // ns
    while (rtController_GetRunningStatus())
    {
        ret = rt_task_wait_period(NULL);
        if (ret)
        {
            rt_printf("EC_CONTROLLER : Erreur lors de rt_task_wait_period, code %d\n",ret);
            break;
        }


        switch( controller.mode)
        {

        /* Mode Attente */
        case WAIT_MODE:
            cycle_counter++;
            if( cycle_counter == 10)
            {
                rtController_Wait_Proc();
                cycle_counter =0;
                controller.mode = CAL_MODE;
            }
            break;
            /* Mode d'étalonnage du système*/
        case CAL_MODE :
            if ( !rtController_GetCalStatus())
            {
            //                if (rtController_DoCal_Proc() < 0)
            //                    break;
                rtController_SetCalStatus(1);
            }
            else
                controller.mode = DIRECT_CTRL_MODE;
            break;
        case DIRECT_CTRL_MODE:
            rtController_DoDirectCtrl_Proc();
            break;
        case CTRL_INCHWORM_MODE:
            rtController_DoCtrlInchworm_Proc();
            break;
        case CTRL_FLAT_INCHWORM_MODE:
            rtController_DoCtrlFlatInchworm_Proc();
            break;
        case TELEOP_MODE:
            rtController_DoTeleop_Proc();
            break;
        default:
            break;
        }

        /* Gestion des requêtes provenant de l'interface homme-machine*/
        rtProcessCommand();
        rtSendDataToSave();
        rtController_updateController();

    }

    rt_printf(PFX"Fin thread principal.\n");

}





void rtController_stop(int sig)
{


    rt_task_delete(&controller_main_task);
    rt_task_delete(&rtController_SlaveTask);
    /* Fermeture des RT queue*/
    //
    /* Fermeture de la communication RT*/
    rtCloseFIFO();
    /**/
    write_data_log_file();

}




void init_save_data(void)
{
    int i;
    save_data.current_time = rt_timer_read();
    save_data.digital_output = 0;
    for (i = 0;i< 8;i++)
        save_data.analog_input[i] = 0;

}


void init_param_struct(void)
{
    int i;
    for (i = 0; i < NB_ANALOG_OUTPUT;i++)
        controller_parameters.analog_output[i] = 0;
    controller_parameters.analog_output[ANA_OUT_5] = 2.5;
    controller_parameters.analog_output[ANA_OUT_6] = 2.5;
    controller_parameters.reg_dig_output = 0;
}

void init_controller_struct(void)
{
    int i;
    controller.cmd          = CMD_NO_CMD;
    controller.mode         = WAIT_MODE;
    controller.submode      = WAIT_SUBMODE;
    controller.status_word  = 0;
    for( i = 0 ; i< NB_ANALOG_INPUT;i++)
        controller.analog_input[i] = 0;
    for( i = 0 ; i< NB_ANALOG_OUTPUT;i++)
        controller.analog_output[i] = 0;
    controller.analog_output[ANA_OUT_5] = 2.5;
    controller.analog_output[ANA_OUT_6] = 2.5;

    for( i = 0 ; i< NB_COUNTER;i++)
        controller.encoder_value[i] = 0;
    for( i = 0 ; i< NB_DIG_INPUT;i++)
        controller.digital_input[i] = 0;
    controller.digital_output = 0;
  //  controller.digital_output_5V = 0;
    controller.dt = 0;
}


void synch_signal(int sig)
{
    printf("Envoi du signal de synchro\n");

}

/**
  * @fn int main(void)
 * @brief Fonction principale
 * @return
 */
int main(void)
{

    int ret,i;
    sigset_t set;
    int sig;
    struct sigaction usr_action;
    sigset_t block_mask;
    pid_t child_id;

    sigfillset(&block_mask);
    usr_action.sa_handler = synch_signal;
    usr_action.sa_mask = block_mask;
    usr_action.sa_flags = 0;
    sigaction(SIGUSR1, &usr_action, NULL);

    sigemptyset(&set);
    sigaddset(&set, SIGINT);

    signal(SIGTERM, rtController_stop);
    signal(SIGINT,  rtController_stop);
    //signal(SIGHUP,  rtController_stop);//Interruption interactive (Ctrl C ou clavier)
    signal(SIGKILL, rtController_stop);
    /*** Initialisation de la structure*/

    /* Interdit le swapping de mémoire pour ce programme*/
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        exit(-2);
    }
    iopl(3);

    rt_print_auto_init(1);
    /* Intialisation de la structure de données utilisée pour sauvegarder les données*/
    init_save_data();
    /* Intialisation des paramètres de structure controller*/
    init_param_struct();

    /* Lecture du data loggeur*/
    //read_data_log_file();

    /****************************************************************************************/
    if (rtOpenFIFO())
    {
        PRINTF_RTCONTROLLER(PFX"Open rt FIFO failed");
        return 0;
    }

    PRINTF_RTCONTROLLER(PFX"RT FIFO Open.\n");

    PRINTF_RTCONTROLLER(PFX"Reserves an EtherCAT master for realtime operation. \n");

    /*****************************************************************************************/
    master = ecrt_request_master(0);
    if (!master)
    {
        PRINTF_RTCONTROLLER(PFX"Echec requete master\n");
        return 0;
    }
    /*****************************************************************************************/
    PRINTF_RTCONTROLLER(PFX" Create a domain.\n");
    domain = ecrt_master_create_domain(master);
    if ( !domain)
    {
        PRINTF_RTCONTROLLER(PFX"Creation of domain failed\n");
        ecrt_release_master(master);
        return 0;
    }
    /*****************************************************************************************/

    /* Configuration des modules EtherCAT */
    /* Configuration des modules EL4104*/
    if ( EL4104_Config(0, AnaOut1SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaOut1SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration des modules EL3164*/
    if ( EL3164_Config(0, AnaIn1SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaIn1SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration des modules EL1008*/
    if ( EL1008_Config(0, DigIn1SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (DigIn1SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration des modules EL3164*/
    if ( EL3164_Config(1, AnaIn2SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaIn2SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    if ( EL3164_Config(2, AnaIn3SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaIn3SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration des modules EL3104*/
    if ( EL3104_Config(0, AnaIn4SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaIn4SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    if ( EL3104_Config(1, AnaIn5SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaIn5SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }

    /* Configuration des modules EL4132*/
    if ( EL4132_Config(0, AnaOut2SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (AnaOut2SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }

    /* Configuration des modules EL5101 */
    if ( EL5101_Config(0, Counter1SlavePos, EL5101_Enable_C_Reset[0], master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (Counter1SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }

    /* Configuration du module EL2004*/
    if (EL2004_Config(0, DigOut1SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX "Failed to get slave configuration (DigOut3SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration des modules EL1008*/
    if ( EL1008_Config(1, DigIn2SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (DigIn1SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration des modules EL1008*/
    if ( EL1008_Config(3, DigIn3SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX" Failed to get slave configuration (DigIn1SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration du module EL2828*/
    if (EL2828_Config(0, DigOut2SlavePos, master, domain))
    {
        PRINTF_RTCONTROLLER(PFX "Failed to get slave configuration (DigOut4SlavePos).\n");
        ecrt_release_master(master);
        return -1;
    }
    /* Configuration du module EL2124*/
//    if (EL2124_Config(0, DigOut3SlavePos, master, domain))
//    {
//        PRINTF_RTCONTROLLER(PFX "Failed to get slave configuration (DigOut4SlavePos).\n");
//        ecrt_release_master(master);
//        return -1;
//    }
     //Create configuration for bus coupler
    if (!ecrt_master_slave_config(master, ECExtenderSlavePos, Beckhoff_EK1100))
    {
        PRINTF_RTCONTROLLER(PFX "Failed to configure bus coupler\n");
        ecrt_release_master(master);
        return -1;
    }

    PRINTF_RTCONTROLLER("Activating master...\n");
    if (ecrt_master_activate(master))
    {
        ecrt_release_master(master);
        return -1;
    }
    PRINTF_RTCONTROLLER("Domain data initialization ...\n");
    domain_pd = ecrt_domain_data(domain) ;

    /* Initialisation des structures de donn�es des modules Beckhoff*/
    /* Initialisation des compteurs*/
    PRINTF_RTCONTROLLER("Initialisation des compteurs ..\n");
    for ( i = 0 ; i < NB_COUNTER; i++)
        EL5101_Init(i, domain_pd);


    rtController_SetRunningStatus(1);


    ret = rt_task_create(&controller_main_task,
                         "CONTROLLER_MAIN_TASK",
                         TASK_STKSZ,
                         99,
                         TASK_MODE);

    ret = rt_task_start(&controller_main_task,
                        &rtController_main_proc,
                        NULL);


    kill(0, SIGUSR1);

    sigwait(&set, &sig);

    rtController_stop(0);

    return 0;
}


