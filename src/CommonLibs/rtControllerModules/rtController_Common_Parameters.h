#ifndef __RT_CONTROLLER_COMMON_PARAM_H__
#define __RT_CONTROLLER_COMMON_PARAM_H__
typedef unsigned int cmd_type;
typedef unsigned int mode_type;
typedef unsigned int error_type;
/*** MODES ****/
#define WAIT_MODE                   0
#define CAL_MODE                    1
#define DIRECT_CTRL_MODE            2
#define JVEL_PLANNER_MODE           3
#define JPOS_STATIC_MODE            4
#define IDENT_MODE                  5
#define GO_TO_INIT_POS_MODE         6
#define POS_PLANNER_MODE            7
#define CTRL_INCHWORM_MODE          8
#define CTRL_FLAT_INCHWORM_MODE     9
#define TELEOP_MODE                 10


/* MODE DE FONCTIONNEMENT SPECIFIQUE DU MAITRE*/
#define CONNECT_HARDWARE_MODE       10
#define CONFIGURE_HARDWARE_MODE     11
#define EXIT_MODE                   12
#define DISCONNECT_HARDWARE_MODE    13
#define CONTROL_ROBOT_MODE          14

/*** SUBMODES ****/
#define WAIT_SUBMODE                0
#define WAIT_TELEOP_SUBMODE             100
#define INIT_TELEOP_SUBMODE             101
#define INCHWORM_CTRL_TELEOP_SUBMODE    102
#define ORIENTATION_CTRL_TELEOP_SUBMODE 103
#define STOP_TELEOP_SUBMODE             104



#define CMD_NO_CMD              0
#define CMD_DO_CAL              1
#define CMD_WRITE_DIG_OUT       2
#define CMD_READ_DIG_OUT        3
#define CMD_WRITE_ANA_OUT       4
#define CMD_POWER_ON            5
#define CMD_POWER_OFF           6
#define CMD_GET_STATUS          7
#define CMD_INIT_INCHWORM       8
#define CMD_CTRL_INCHWORM       9
#define CMD_STOP_INCHWORM       10
#define CMD_CTRL_FLAT_INCHWORM  11
#define CMD_WRITE_DIG_OUT_5V    12
#define CMD_START_TELEOP        13
#define CMD_STOP_TELEOP         14
#define CMD_EXIT                99

/* COMMANDES */
#define CMD_CONNECT_HARDWARE        10
#define CMD_CONFIGURE_HARDWARE      11
#define CMD_DISCONNECT_HARDWARE     12

#define CMD_OPEN_CHUCK             13
#define CMD_CLOSE_CHUCK             14
#define CMD_CONNECT_MASTER_DEVICE   18
#define CMD_SWITCH_TO_SLAVE         19


/*ERRORS*/
#define ERROR_READ_ANALOG_INPUT     -1
#define ERROR_READ_DIG_INPUT        -2
#define ERROR_WRITE_DIG_OUTPUT      -3
#define ERROR_WRITE_ANALOG_OUTPUT   -4
#define ERROR_FIFO_OPEN_FAILED      -5
#endif
