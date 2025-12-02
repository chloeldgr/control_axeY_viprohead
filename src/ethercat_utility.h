#ifndef CONTROL_SEGMENTED_TRAJ_STATE_MACHINE_H
#define CONTROL_SEGMENTED_TRAJ_STATE_MACHINE_H


#define _GNU_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <sched.h>
#include <pthread.h>
#include <stdbool.h>
#include "ecrt.h"


/* -------------------------------------------------------------------------- */
/*                               GLOBAL CONSTANTS                             */
/* -------------------------------------------------------------------------- */

#define PERIOD_NS 1000000L               // 1 ms
#define NSEC_PER_SEC 1000000000L
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

#define FACTOR 16.3835                   // steps/s -> EL7031 PDO scaling

#define EPSILON 0.100

#define MM_TO_UM(x) ((int32_t)((x) * 1000.0))
#define UM_TO_MM(x) ((double)(x) / 1000.0)

/* CiA-402 statusword bits */
#define SW_TARGET_REACHED 0x0400

/* Vendor / Product codes */
#define ED1F               0x0000aaaa, 0x00000005
#define Beckhoff_EK1100    0x00000002, 0x01c33052
#define Beckhoff_EL7031    0x00000002, 0x1b773052

/* EtherCAT positions */
#define BusCouplerPos 0, 4
#define AxisYPos      0, 3
#define ExtruderPos   0, 5

/* -------------------------------------------------------------------------- */
/*                                   ENUMS                                    */
/* -------------------------------------------------------------------------- */

typedef enum {
    CYCLE_IDLE = 0,
    CYCLE_HOMING,
    CYCLE_SEGMENTED_TRAJECTORY,
    CYCLE_ERROR,
    CYCLE_SHUTDOWN
} CycleMode;


/* -------------------------------------------------------------------------- */
/*                                 STRUCTURES                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
    double y_position_mm;
    double user_velocity_steps_per_s;
    bool motion_started;
    bool motion_complete;
    bool system_error;
    uint32_t error_code;
} SystemState;

typedef struct {
    FILE *log_extrusion_file;
    FILE *log_position_file;
} LogFiles;

typedef struct {
    ec_master_t *master;
    ec_domain_t *domain1;
    uint8_t *domain1_pd;
} EtherCATContext;


typedef struct {
    ec_master_state_t master_state;
    ec_domain_state_t domain1_state;
} EtherCATState;

typedef struct {
    ec_slave_config_t *sc_extruder;
    ec_slave_config_state_t sc_extruder_state;
    unsigned int off_extruder_velocity;
    unsigned int off_extruder_control_byte;
    unsigned int off_extruder_status_byte;
} Extruder;

typedef struct {
    ec_slave_config_t *sc_axis;
    ec_slave_config_state_t sc_axis_state;
    unsigned int off_axis_pos_actual;
    unsigned int off_axis_statusword;
    unsigned int off_axis_controlword;
    unsigned int off_axis_target;
    int32_t axis_target_pos_um;
} Axis;

typedef struct {
    double dist_avance_vide1;
    double dist_extru_lente;
    double dist_extru_rapide;
    double dist_avance_vide2;
    double axeY_vel_mm_s;
    double extru_lente;
    double extru_rapide;
} TrajectoryParams;



/* -------------------------------------------------------------------------- */
/*                               GLOBAL VARIABLES                              */
/* -------------------------------------------------------------------------- */

extern CycleMode cycleMode;

extern unsigned int counter = 0;

extern volatile sig_atomic_t stop = 0;
extern SystemState systemState = {0};
extern LogFiles logsFiles = {NULL, NULL};
extern EtherCATContext ecContext = {NULL, NULL, NULL, NULL, NULL};
extern EtherCATState ecState = {NULL, NULL, NULL} ;
extern Extruder vExtruder = {NULL, NULL, 0, 0, 0};
extern Axis yAxis = {NULL, NULL, 0, 0, 0, 0, 0};
extern TrajectoryParams trajParams = {
    .dist_avance_vide1 = 10.0,
    .dist_extru_lente = 50.0,
    .dist_extru_rapide = 50.0,
    .dist_avance_vide2 = 10.0,
    .axeY_vel_mm_s = 7.0,
    .extru_lente = 186.5,
    .extru_rapide = 266.43,
};

/* -------------------------------------------------------------------------- */
/*                           FUNCTION DECLARATIONS                             */
/* -------------------------------------------------------------------------- */

/* State checking */
void check_domain_state(EtherCATContext ec_context, EtherCATState ec_state);
void check_master_state(EtherCATContext ec_context, EtherCATState ec_state) ;
void check_extruder_config_states(Extruder extruder_to_check);
void check_axis_config_states(Axis axis_to_check);
void ec_check_and_print_states(EtherCATContext ec_context, EtherCATState ec_state, Extruder extruder_to_check, Axis axis_to_check) {

/* SDO operations */
int write_sdo(EtherCATContext ec_context, uint16_t slave, uint16_t index, uint8_t sub, const void *val, size_t size);
int read_sdo(EtherCATContext ec_context, uint16_t slave, uint16_t index, uint8_t sub, void *target, size_t size);

/* EtherCAT control functions */
void sigint_handler(int s);

void ec_send_controlword(EtherCATContext ec_context, Axis axis_to_send_cw, uint16_t controlword);
void ec_set_target_position(EtherCATContext ec_context, Axis axis_to_move, int32_t target_um);
bool is_position_reached(double cur, double target);

void ec_enable_extruder(EtherCATContext ec_context, Extruder extruder_to_enable, bool enable);
void ec_set_extruder_velocity(EtherCATContext ec_context, Extruder extruder_to_set_velocity, double velocity_steps_per_s);

/* Logging */
void log_position_and_extrusion(LogFiles log_files, double timestamp, double position_mm, double extrusion_steps_per_s);

/* Actions on single axis */
int config_and_launch_single_axis(EtherCATContext ec_context, Axis axis_to_config);
void ec_shutdown_single_axis(EtherCATContext ec_context, Axis axis_to_shutdown);

/* Homing */
void start_homing_sequence(EtherCATContext ec_context, Axis axis_to_homing);
void homing_sequence(EtherCATContext ec_context, Axis axis_to_homing, CycleMode cycle_mode ); 
/* Cyclic task */
void cyclic_task(EtherCATContext ec_context, EtherCATState ec_state, Extruder extruder_to_use, Axis axis_to_move, SystemState system_state, CycleMode cycle_mode, TrajectoryParams traj_segmented, LogFiles log_files);

/* Supervisor thread */
void* supervisor_thread(void *arg);

#endif // CONTROL_SEGMENTED_TRAJ_STATE_MACHINE_H