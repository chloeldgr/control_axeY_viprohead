// control_vipro_axeY_state_machine.c 
//
/* Controle de l'axe Y (ED1F CoE drive) et de l'extrudeur (EL7031) via etherCAT en C pur
 * Fonctionnalités :
 *  - Initialisation EtherCAT complète
 *  - Séquence CiA-402 de démarrage du drive
 *  - Lancement d’une trajectoire en mm
 *  - Gestion de l’extrusion selon position réelle
 */

// --- Pour build aller dans le dossier build et faire : ---
// gcc -o control_vipro_axeY_state_machine ../src/control_vipro_axeY_state_machine.c -lrt -pthread -L/usr/local/etherlab/lib -lethercat
// --- Pour lancer le code: ---
// sudo LD_LIBRARY_PATH=/usr/local/etherlab/lib ./control_vipro_axeY_state_machine

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
/* ------------------------------------------------------------- */
/*              DEFINITIONS DES VARIABLES GLOBALES               */
/* ------------------------------------------------------------- */

#define PERIOD_NS 1000000L  // 1 ms
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS) // iterations/s

#define FACTOR 16.3835      // factor steps/s --> PDO (extruder)
#define POLL_INTERVAL_US 50000 // 50 ms

/* Trajectory paramters in mm */
#define DIST_AVANCE_VIDE1 10.0
#define DIST_EXTRU_LENTE 50.0
#define DIST_EXTRU_RAPIDE 50.0
#define DIST_AVANCE_VIDE2 10.0
#define AXEY_VEL_MM_S 7.0

/* paramètres debounce */
#define ZERO_DEBOUNCE_CYCLES 5
#define TARGET_DEBOUNCE_CYCLES 5
#define ZERO_TOL_MM 0.05
#define TARGET_TOL_MM 0.05

/* Extrusion parameters in steps/s */
#define EXTRU_LENTE 186.5
#define EXTRU_RAPIDE 266.43


/* Conversion mm <--> um parameters */
#define MM_TO_UM(x) ((int32_t)(x)*1000.0)
#define UM_TO_MM(x) ((double)(x)/1000.0)


/* CiA-402 Statusword bits */
#define SW_TARGET_REACHED 0x0400

/* Shared variables */
volatile sig_atomic_t stop = 0;
void sigint_handler(int s) { stop = 1; }

double y_position_mm = 0.0 ;
double user_velocity_steps_per_s = 0.0 ; // actual command (steps/s pour extrudeur)

bool motion_complete = false;
bool zero_reached = false;
bool motion_started = false ;

FILE *log_extrusion_file = NULL;   // log des changements extrusion
FILE *log_position_file  = NULL;   // log des positions toutes les 1 ms


/* ------------------------------------------------------------- */
/*          DEFINITIONS ETATS DE LA MACHINE D'ETAT               */
/* ------------------------------------------------------------- */

typedef enum {
    IDLE,
    MOVE_TO_ZERO,
    SEGMENTED_TRAJECTORY,
    ERROR,
    SHUTDOWN
} SystemState;
volatile SystemState current_state = IDLE;
volatile SystemState next_state = IDLE;



/* ------------------------------------------------------------- */
/*                     PROTOTYPES DE FONCTIONS                   */
/* ------------------------------------------------------------- */
void sigint_handler(int s) ;
void check_domain1_state(void);
void check_master_state(void);
void check_slave_config_states(void);
void cyclic_task(void);  // <-- Ajoutez cette ligne
void move_to_zero(void);
int init_system(void);
void cleanup_system(void);
void handle_segmented_trajectory(void);
int send_controlword_sequence_via_pdo(void);
int write_sdo(uint16_t slave, uint16_t index, uint8_t sub, const void *val, size_t size);
static int read_sdo(uint16_t slave, uint16_t index, uint8_t sub, void *target, size_t size);
void *supervisor_thread(void *arg);


/* -------------------------------------------------------------------------- */
/*                             OFFSETS PDO REGISTERS                          */
/* -------------------------------------------------------------------------- */
/* PDO registration offsets (filled by ecrt) */
/* EL7031 --> Extruder */
static unsigned int off_extruder_velocity;
static unsigned int off_extruder_control_byte;
static unsigned int off_extruder_status_byte;
/* ED1F --> Axis Y */
static unsigned int off_axisY_pos_actual;   // 32-bit position actual (µm)
static unsigned int off_axisY_statusword;   // 16-bit statusword
static unsigned int off_axisY_controlword;   // 16-bit controlword (RxPDO)
static unsigned int off_axisY_target; // 32-bit target position (RxPDO)

/* additionally keep a global copy of the target to write into PDO */
static int32_t axisY_target_pos_um = 0;




/* -------------------------------------------------------------- */
/*                   EtherCAT Master Config                       */
/* -------------------------------------------------------------- */
// Master / domain / slave configs (globales)
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_extruder = NULL; // EL7031
static ec_slave_config_state_t sc_extruder_state = {};

static ec_slave_config_t *sc_y = NULL; // ED1F 
static ec_slave_config_state_t sc_y_state = {};

// PD buffer
static uint8_t *domain1_pd = NULL;


/* Vendor/Product pairs */
#define ED1F 0x0000aaaa, 0x00000005  // Drive for Y axis (slave 3)
#define Beckhoff_EK1100 0x00000002, 0x01c33052 // Bus Coupler
#define Beckhoff_EL7031 0x00000002, 0x1b773052 // Drive for Viprohead extruder

/* Bus positions (Master, position) */
#define BusCouplerPos    0, 4
#define AxisYPos         0, 3  // slave 3 = Axis Y (read position)
#define ExtruderPos      0, 5  // slave 5 = EL7031

/* -------------------------------------------------------------------------- */
/*                              PDO entry infos                               */
/* Here we will register:                                  
   - For Axis Y (slave 3 ED1F) we will register position actual value (0x6064)
     and statusword (0x6041) from the TxPDO (so we can read them).
   - For extruder (EL7031) (control byte, velocity, status byte). */

/* -------------------------------------------------------------------------- */
/*                            STRUCTURE PDO                                   */
/* -------------------------------------------------------------------------- */
/* SLAVE 3 CONFIGURATION (ED1F CoE Drive) */
/* Master 0, Slave 3, "ED1F CoE Drive"
 * Vendor ID:       0x0000aaaa
 * Product code:    0x00000005
 * Revision number: 0x00010000
 */
ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    /* RxPDO (outputs) */
    {0x6040, 0x00, 16}, // Controlword (non utilisé en runtime)
    {0x6060, 0x00, 8},  // Mode of operation (non utilisé en runtime)
    {0x607a, 0x00, 32}, // Target position (non utilisé en runtime)
    /* TxPDO (inputs) - ce qu'on lit */
    {0x6041, 0x00, 16}, // Statusword
    {0x6061, 0x00, 8},  // Mode of operation display
    {0x6064, 0x00, 32}, // Position actual value
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 3, slave_3_pdo_entries + 0}, // RxPDO
    {0x1a00, 3, slave_3_pdo_entries + 3}, // TxPDO
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_3_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_3_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* SLAVE 5 CONFIGURATION (EL7031) */
/* Master 0, Slave 5, "EL7031"
 * Vendor ID:       0x00000002
 * Product code:    0x1b773052
 * Revision number: 0x001a0000
 */
ec_pdo_entry_info_t el7031_pdo_entries[] = {
    {0x0000, 0x00, 1}, /* Gap */
    {0x7000, 0x02, 1}, /* Enable latch extern on positive edge */
    {0x7000, 0x03, 1}, /* Set counter */
    {0x7000, 0x04, 1}, /* Enable latch extern on negative edge */
    {0x0000, 0x00, 4}, /* Gap */
    {0x0000, 0x00, 8}, /* Gap */
    {0x7000, 0x11, 16}, /* Set counter value */
    {0x7010, 0x01, 1}, /* Enable */
    {0x7010, 0x02, 1}, /* Reset */
    {0x7010, 0x03, 1}, /* Reduce torque */
    {0x0000, 0x00, 5}, /* Gap */
    {0x0000, 0x00, 8}, /* Gap */
    {0x7010, 0x21, 16}, /* Velocity */
    {0x6010, 0x01, 1}, /* Ready to enable */
    {0x6010, 0x02, 1}, /* Ready */
    {0x6010, 0x03, 1}, /* Warning */
    {0x6010, 0x04, 1}, /* Error */
    {0x6010, 0x05, 1}, /* Moving positive */
    {0x6010, 0x06, 1}, /* Moving negative */
    {0x6010, 0x07, 1}, /* Torque reduced */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 3}, /* Gap */
    {0x6010, 0x0c, 1}, /* Digital input 1 */
    {0x6010, 0x0d, 1}, /* Digital input 2 */
    {0x6010, 0x0e, 1}, /* Sync error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x6010, 0x10, 1}, /* TxPDO Toggle */
};
ec_pdo_info_t el7031_pdos[] = {
    {0x1600, 7, el7031_pdo_entries + 0}, /* ENC RxPDO-Map Control compact */
    {0x1602, 5, el7031_pdo_entries + 7}, /* STM RxPDO-Map Control */
    {0x1604, 1, el7031_pdo_entries + 12}, /* STM RxPDO-Map Velocity */
    {0x1a03, 14, el7031_pdo_entries + 13}, /* STM TxPDO-Map Status */
};
ec_sync_info_t el7031_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 3, el7031_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, el7031_pdos + 3, EC_WD_DISABLE},
    {0xff}
};
// Mapping PDO 
/* PDO registration table: register PDO entries we need into domain1 */ 
const static ec_pdo_entry_reg_t domain1_regs[] = { 
    /* Axis Y (ED1F drive at bus position 3) - read-only entries (TxPDO) */ 
    { AxisYPos, ED1F, 0x6064, 0x00, &off_axisY_pos_actual }, 
    { AxisYPos, ED1F, 0x6041, 0x00, &off_axisY_statusword }, 
    { AxisYPos, ED1F, 0x6040, 0x00, &off_axisY_controlword }, 
    { AxisYPos, ED1F, 0x607A, 0x00, &off_axisY_target}, 
    /* Extruder EL7031 */ 
    { ExtruderPos, Beckhoff_EL7031, 0x7010, 0x01, &off_extruder_control_byte }, // control byte 
    { ExtruderPos, Beckhoff_EL7031, 0x7010, 0x21, &off_extruder_velocity }, // velocity (16-bit in your mapping) 
    { ExtruderPos, Beckhoff_EL7031, 0x6010, 0x01, &off_extruder_status_byte }, // status byte 
    {} /* terminator */ 
    };


/* --------------------------------------------------------------- */
/*                  STATE CHECKING HELPERS                         */
/* --------------------------------------------------------------- */
void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    domain1_state = ds;
}

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    master_state = ms;
}

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc_extruder, &s);
    if (s.al_state != sc_extruder_state.al_state) {
        printf("Extruder: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_extruder_state.online) {
        printf("Extruder: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_extruder_state.operational) {
        printf("Extruder: %soperational.\n", s.operational ? "" : "Not ");
    }
    sc_extruder_state = s;


    ecrt_slave_config_state(sc_y, &s);
    if (s.al_state != sc_y_state.al_state) {
        printf("Axis Y: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_y_state.online) {
        printf("Axis Y: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_y_state.operational) {
        printf("Axis Y: %soperational.\n", s.operational ? "" : "Not ");
    }
    sc_y_state = s;
}


/* -------------------------------------------------------------------------- */ 
/*                          FONCTIONS SDO                                     */ 
/* -------------------------------------------------------------------------- */
int write_sdo(uint16_t slave, uint16_t index, uint8_t sub, const void *val, size_t size) {
    uint32_t abort = 0;
    int ret = ecrt_master_sdo_download(master, slave, index, sub, (uint8_t *)val, size, &abort);
    if (ret < 0)
        fprintf(stderr, "Erreur SDO 0x%04X:%02X (abort=0x%08X)\n", index, sub, abort);
    return ret;
}

// int write_sdo(uint16_t slave, uint16_t index, uint8_t sub, const void *val, size_t size) {
//     uint32_t abort=0 ; 
//     printf(" Écriture SDO esclave %u: 0x%04X:%02X... ", slave, index, sub); 
    
//     /* Blocking SIGINT during SDO operation to avoid EINTR caused by Ctrl+C */ 
//     sigset_t newset, oldset; sigemptyset(&newset); sigaddset(&newset, SIGINT); 
//     if (pthread_sigmask(SIG_BLOCK, &newset, &oldset) != 0) { perror("pthread_sigmask"); 
//     } 

//     int max_attempts = 5; 
//     int attempt = 0; 
//     int ret = -1; 
//     while (attempt < max_attempts) { 
//         ret = ecrt_master_sdo_download(master, slave, index, sub, (uint8_t *)val, size, &abort); 
//         if (ret >= 0) { 
//             break; 
//         } 
//         if (errno == EINTR) { 
//             attempt++; fprintf(stderr, "SDO interrupted, retry %d/%d...\n", attempt, max_attempts); 
//             usleep(100000); /* 100 ms */ 
//             continue; 
//         } else { 
//             fprintf(stderr, "Failed download SDO (abort=0x%08X) (ret=%d errno=%d)\n", abort, ret, errno); 
//             break; 
//         } 
//     } 
    
//     /* Restore sigmask */ 
//     if (pthread_sigmask(SIG_SETMASK, &oldset, NULL) != 0) { 
//         perror("pthread_sigmask restore"); 
//     } 
    
//     if (ret < 0) { 
//         return ret; 
//     } 
    
//     printf("OK\n"); 
//     return ret; 
// } 

// static int read_sdo(uint16_t slave, uint16_t index, uint8_t sub, void *target, size_t size) { 
//     uint32_t abort; size_t result_size; int ret = ecrt_master_sdo_upload(master, slave, index, sub, (uint8_t *)target, size, &result_size, &abort); 
//     if (ret < 0) { 
//         fprintf(stderr, "Erreur upload SDO 0x%04X:%02X (abort=0x%08X)\n", index, sub, abort); 
//     } 
// return ret; 
// }



/* -------------------------------------------------------------------------- */
/*          CONFIGURATION AND LAUNCH Y-AXIS (via PDO controlword)             */
/* -------------------------------------------------------------------------- */
/* Activation sequence via PDO: send controlword by PDO (shutdown/switch-on/enable) */
int send_controlword_sequence_via_pdo() {
    if (!domain1_pd || off_axisY_controlword == 0) {
        fprintf(stderr, "[SEND_CW] Controlword PDO offset invalide\n");
        return -1;
    }
    /* If target offset is available, write the target position into PDO before enabling */
    if (off_axisY_target != 0) {
        EC_WRITE_S32(domain1_pd + off_axisY_target, axisY_target_pos_um);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(200000);
        printf("[SEND_CW] Cible écrite: %d µm.\n", axisY_target_pos_um);
    }

    // minimal sequence: shutdown -> switch on -> enable -> start
    uint16_t cw = 0x0006; // Shutdown
    EC_WRITE_U16(domain1_pd + off_axisY_controlword, cw);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(200000);

    cw = 0x0007; // Switch on
    EC_WRITE_U16(domain1_pd + off_axisY_controlword, cw);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(100000);

    cw = 0x000F; // Enable operation
    EC_WRITE_U16(domain1_pd + off_axisY_controlword, cw);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(100000);

    /* Start motion: set new setpoint + start */
    cw = 0x009B;
    EC_WRITE_U16(domain1_pd + off_axisY_controlword, cw);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(100000);

    return 0;
}


/* ------------------------------------------------------------- */
/*                  FONCTION INITIALISATION                      */
/* ------------------------------------------------------------- */
int init_system(void) {
    signal(SIGINT, sigint_handler);

    printf("[INIT] Ouverture des logs...\n");
    log_extrusion_file = fopen("extrusion_events.csv", "w");
    if (!log_extrusion_file) {
        perror("[INIT] Erreur : log extrusion");
        exit(1);
    }
    fprintf(log_extrusion_file, "timestamp_s,y_position_mm,extrusion_steps_s\n");
    //printf("[INIT] Log extrusion ouvert.\n");

    log_position_file = fopen("position_log.csv", "w");
    if (!log_position_file) {
        perror("[INIT] Erreur : log position");
        exit(1);
    }
    fprintf(log_position_file, "timestamp_s,y_position_mm,extrusion_steps_s\n");
    printf("[INIT] Log position ouvert.\n");

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║   EtherCAT Control - Axe Y + Extrudeur (Méthode Shell)     ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    /* --- Initialisation master EtherCAT (now take ownership of master) --- */
    printf("=== Initialisation EtherCAT ===\n");
    master = ecrt_request_master(0);
    if (!master) {
        fprintf(stderr, "[INIT] Erreur : impossible d'obtenir le master EtherCAT.\n");
        return 1;
    }
    printf("[INIT] Master EtherCAT obtenu.\n");

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) {
        fprintf(stderr, "[INIT] Erreur: création du domaine échouée.\n");
        return 1;
    }

    /* --- Configuration slaves --- */    
    /* Configure EK1100 */
    printf("[INIT] Configuring EK1100...\n");
    if (!ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100)){
        fprintf(stderr, "[INIT] Failed to configure EK1100.\n");
        return -1;
    }
    // Configuration ED1F (Axe Y)
    printf("[INIT] Configuration ED1F (Axe Y)...\n");
    sc_y = ecrt_master_slave_config(master, AxisYPos, ED1F);
    if (!sc_y) {
        fprintf(stderr, "[INIT] Erreur : Failed to configure ED1F.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_y, EC_END, slave_3_syncs)) {
        fprintf(stderr, "[INIT] Erreur : Failed to configure PDOs ED1F.\n");
        return -1;
    }
    // Configuration EL7031 (Extrudeur)
    printf("[INIT] Configuration EL7031 (Extruder)...\n");
    sc_extruder = ecrt_master_slave_config(master, ExtruderPos, Beckhoff_EL7031);
    if (!sc_extruder) {
        fprintf(stderr, "[INIT] Erreur : Failed to configure EL7031.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(sc_extruder, EC_END, el7031_syncs)) {
        fprintf(stderr, "[INIT] Erreur : Failed to configure PDOs EL7031.\n");
        return -1;
    }

    // Enregistrement PDO
    printf("[INIT] Saving PDO...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        fprintf(stderr, "[INIT] Erreur : Failed to save PDO!\n");
        return -1;
    }


    /* ------------------------------------------------------------
     * SDO initial configuration BEFORE activating master
     * - 0x6060 : Mode of operation (Profile Position Mode)
     * - 0x6081 : Profile velocity (optional)
     * - 0x607A : target (OPTIONAL — ton ancien code l'écrivait)
     * -----------------------------------------------------------*/

    printf("\n=== [INIT] Configuration SDO avant activation ===\n");

    /* Configuration SDO avant activation du master */
    uint32_t vel_um_s = (uint32_t)(AXEY_VEL_MM_S * 1000.0);
    int8_t mode_ppm = 1; // Profile Position Mode

    printf("[INIT] Écriture SDO 0x6060 (mode)... ");
    if (write_sdo(3, 0x6060, 0x00, &mode_ppm, sizeof(mode_ppm)) < 0) {
        fprintf(stderr, "[INIT] ERROR: Failed to configure mode (SDO)\n");
    } else {
        printf("OK.\n");
    }
    usleep(100000);


    printf("[INIT] Écriture SDO 0x6081 (vitesse)... ");
    if (write_sdo(3, 0x6081, 0x00, &vel_um_s, sizeof(vel_um_s)) < 0) {
        fprintf(stderr, "[INIT]ERROR: Failed to configure target velocity (SDO)\n");
    } else {
        printf("OK.\n");
    }

    usleep(100000);


    // /* --- Option: écrire la target initiale via SDO
    //  * Ton code de référence faisait cela (cible = somme des segments).
    //  * Si tu veux NE PAS l'écrire à l'init, commente le bloc suivant.
    //  */
    // int32_t target_pos_um = 0 ;
    // axisY_target_pos_um = target_pos_um;

    // printf("[INIT] (OPTION) Ecriture SDO 0x607A (target) ... ");
    // if (write_sdo(3, 0x607A, 0x00, &target_pos_um, sizeof(target_pos_um)) < 0) {
    //     fprintf(stderr, "[INIT] WARNING: Failed to configure target position (SDO)\n");
    // } else {
    //     printf("OK.\n");
    // }
    // usleep(100000);

    printf("[INIT] SDO initialisation done.\n");

    // Activation du master
    printf("[INIT] Activation du master...\n");
    if (ecrt_master_activate(master)) {
        fprintf(stderr, "[INIT] Erreur : Failed activating master!\n");
        exit(1);
    }
    printf("[INIT] Master activé.\n");

    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd) {
        fprintf(stderr, "[INIT] Erreur : Failed returning domain data!\n");
        exit(1);
    }
    printf("[INIT] Domain activé.\n");

    printf("[INIT] Offsets PDO:Y_pos=%u Y_status=%u Y_cw=%u Y_target=%u Ext_ctl=%u Ext_vel=%u\n", off_axisY_pos_actual, off_axisY_statusword, off_axisY_controlword, off_axisY_target, off_extruder_control_byte, off_extruder_velocity);

    // Run PDO loop before all SDO configuration
    printf("[INIT] Exchange of initial PDO (3 secondes)...\n");
    struct timespec wakeup;
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    
    for (int i = 0; i < 3000; i++) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        
        wakeup.tv_nsec += PERIOD_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) {
            wakeup.tv_nsec -= NSEC_PER_SEC;
            wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);
    }
    printf("[INIT] Communication done!") ;
    printf("[INIT] Système prêt. État initial: IDLE.\n");
    return 0 ;
}

/* ------------------------------------------------------------- */
/*                  FONCTION TACHE CYCLIQUE                      */
/* ------------------------------------------------------------- *
 *
 *  Fonction exécutée toutes les 1 ms :
 *   - Lecture PDO
 *   - Mise à jour position axe Y
 *   - Calcul extrusion en fonction de extrusion_cmd
 *   - Logging CSV (position + extrusion)
 *   - Application du controlword cible (controlword_to_send)
 *   - Application du target position via PDO
 *   - Vérification état esclave / erreurs
 *
 *  La logique machine d’état (MoveToZero / SegmentedTrajectory)
 *  est DANS les fonctions séparées move_to_zero() et
 *  handle_segmented_trajectory().
 * ---------------------------------------------------------*/
static unsigned int counter = 0;
void cyclic_task() {

 // --- Receive PDOs, process domain ---
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // --- Appel des fonctions de vérification (toutes les 1000 itérations) ---
    if (counter == 0) {
        counter = 1000;
        check_domain1_state();
        check_master_state();
        check_slave_config_states();
    }
    counter--;


    // --- Read Axis Y position if offset valid ---
    int32_t pos_um = 0;
    if (off_axisY_pos_actual) {
        pos_um = EC_READ_S32(domain1_pd + off_axisY_pos_actual);
        y_position_mm = UM_TO_MM(pos_um);
    }

    uint16_t axis_status = off_axisY_statusword ? EC_READ_U16(domain1_pd + off_axisY_statusword) : 0;
    // // --- Read statusword Y ---
    // uint16_t axis_status = 0;
    // if (off_axisY_statusword != 0) {
    //     axis_status = EC_READ_U16(domain1_pd + off_axisY_statusword);
    // }

    // --- Gestion de l'état MOVE_TO_ZERO ---
    // --- State-specific PDO writes (non intrusive) ---
    if (current_state == MOVE_TO_ZERO) {
        // ensure target = 0 and start bit are periodically written
        if (off_axisY_target) {
            EC_WRITE_S32(domain1_pd + off_axisY_target, 0);
        }
        if (off_axisY_controlword) {
            EC_WRITE_U16(domain1_pd + off_axisY_controlword, 0x009B);
        }

        // debounced detection
        static int zero_confirm_count = 0;
        if ((axis_status & SW_TARGET_REACHED) || fabs(y_position_mm) <= ZERO_TOL_MM) {
            zero_confirm_count++;
            if (zero_confirm_count >= ZERO_DEBOUNCE_CYCLES) {
                zero_reached = true;
                zero_confirm_count = 0;
            }
        } else {
            zero_confirm_count = 0;
        }
    }
    // --- Gestion de l'état SEGMENTED_TRAJECTORY ---
    if (current_state == SEGMENTED_TRAJECTORY) {
        // write target each cycle and start
        if (off_axisY_target != 0) {
            EC_WRITE_S32(domain1_pd + off_axisY_target, axisY_target_pos_um);
        }
        if (off_axisY_controlword != 0) {
            EC_WRITE_U16(domain1_pd + off_axisY_controlword, 0x009B);
        }

        // detect end of trajectory (debounced)
        static int target_confirm_count = 0;
        if ((axis_status & SW_TARGET_REACHED) || fabs(y_position_mm - UM_TO_MM(axisY_target_pos_um)) <= TARGET_TOL_MM) {
            target_confirm_count++;
            if (target_confirm_count >= TARGET_DEBOUNCE_CYCLES) {
                motion_complete = true;
                target_confirm_count = 0;
            }
        } else {
            target_confirm_count = 0;
        }
        
        // --- DÉTECTION DES SEUILS D’EXTRUSION (1 ms) ---
        static int etape = 0;
        static double last_extrusion = -1.0;
        double seuil1 = DIST_AVANCE_VIDE1;
        double seuil2 = DIST_AVANCE_VIDE1 + DIST_EXTRU_LENTE;
        double seuil3 = DIST_AVANCE_VIDE1 + DIST_EXTRU_LENTE + DIST_EXTRU_RAPIDE;

        switch (etape) {
            case 0:
                if (y_position_mm >= seuil1) {
                    user_velocity_steps_per_s = EXTRU_LENTE;
                    etape = 1;
                    //printf("[CYCLIC] Changement d'étape : EXTRU_LENTE (%.1f steps/s) à Y=%.2f mm\n", user_velocity_steps_per_s, y);
                }
                break;
            case 1:
                if (y_position_mm >= seuil2) {
                    user_velocity_steps_per_s = EXTRU_RAPIDE;
                    etape = 2;
                    //printf("[CYCLIC] Changement d'étape : EXTRU_RAPIDE (%.1f steps/s) à Y=%.2f mm\n", user_velocity_steps_per_s, y);
                }
                break;
            case 2:
                if (y_position_mm >= seuil3) {
                    user_velocity_steps_per_s = 0.0;
                    etape = 3;
                    //printf("[CYCLIC] Changement d'étape : ARRÊT EXTRUSION (%.1f steps/s) à Y=%.2f mm\n", user_velocity_steps_per_s, y);
                }
                break;
        }

        // --- Log des changements d'extrusion ---
        if (fabs(user_velocity_steps_per_s - last_extrusion) > 0.1) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            double t = ts.tv_sec + ts.tv_nsec * 1e-9;
            if (log_extrusion_file) {
                fprintf(log_extrusion_file, "%f,%.3f,%.1f\n", t, y_position_mm, user_velocity_steps_per_s);
                fflush(log_extrusion_file);
            }
            last_extrusion = user_velocity_steps_per_s;
            printf("[EVENT] Extrusion=%.1f steps/s à Y=%.2f mm\n", user_velocity_steps_per_s, y_position_mm);
        }
    }
        // // fallback detection by position
        // if (y_position_mm >= UM_TO_MM(axisY_target_pos_um) - 0.01) {
        //     motion_complete = true;
        //     printf("[INFO] End of the trajectory detected by position (%.2f mm).\n", y_position_mm);
        // }

    // --- Log de la position à chaque cycle ---
    if (log_position_file) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double t = ts.tv_sec + ts.tv_nsec * 1e-9;
        fprintf(log_position_file, "%f,%.3f,%.1f\n", t, y_position_mm, user_velocity_steps_per_s);
    }

    // --- Write extrusion speed --- 
    uint16_t scaled = (uint16_t)(user_velocity_steps_per_s * FACTOR);
    if (off_extruder_control_byte != 0){
        EC_WRITE_U8(domain1_pd + off_extruder_control_byte, 1);
    }
    if (off_extruder_velocity != 0){
        EC_WRITE_U16(domain1_pd + off_extruder_velocity, scaled);
    }

    // --- Envoi des PDO ---
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

}

/* ------------------------------------------------------------- */
/*                      FONCTION MOVE_TO_ZERO                    */
/* ------------------------------------------------------------- */
void move_to_zero() {
    if (!domain1_pd || off_axisY_pos_actual == 0 || off_axisY_controlword == 0 || off_axisY_target == 0) {
        fprintf(stderr, "[MOVE_TO_ZERO] Offsets PDO invalides pour move_to_zero()\n");
        return;
    }
    int32_t cur_pos_um = EC_READ_S32(domain1_pd + off_axisY_pos_actual);
    double cur_mm = UM_TO_MM(cur_pos_um);
    printf("[MOVE_TO_ZERO] Position actuelle: %.3f mm\n", cur_mm);

    if (cur_mm <= 0.01) {
        printf("[MOVE_TO_ZERO] Déjà en position 0.\n");
        zero_reached = true;
        return;
    }

    printf("[MOVE_TO_ZERO] Écriture de la cible (0 mm) dans PDO...\n");
    EC_WRITE_S32(domain1_pd + off_axisY_target, 0);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    usleep(50000);

    printf("[MOVE_TO_ZERO] Envoi de la séquence CiA-402...\n");
    if (send_controlword_sequence_via_pdo() != 0) {
        fprintf(stderr, "[MOVE_TO_ZERO] ERREUR: échec de l'envoi de la séquence de contrôle.\n");
        next_state = ERROR;
        return;
    }

    printf("[MOVE_TO_ZERO] Déplacement en cours...\n");
    int zero_reached_count = 0;
    while (!stop) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        cur_pos_um = EC_READ_S32(domain1_pd + off_axisY_pos_actual);
        cur_mm = UM_TO_MM(cur_pos_um);
        uint16_t axis_status = EC_READ_U16(domain1_pd + off_axisY_statusword);

        printf("[MOVE_TO_ZERO] Position actuelle: %.3f mm | Statusword: 0x%04X\n", cur_mm, axis_status);

        if ((axis_status & SW_TARGET_REACHED) || cur_mm <= 0.5){
            zero_reached_count++;
            printf("[MOVE_TO_ZERO] Position proche de 0 (tolérance: %.3f mm). Compteur: %d/5\n", cur_mm, zero_reached_count);
        } else {
            zero_reached_count = 0;
        }

        if (zero_reached_count >= 5) {
            zero_reached = true;
            printf("[MOVE_TO_ZERO] Position 0 confirmée.\n");
            break;
        }

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(5000);
    }
}


/* ------------------------------------------------------------- */
/*               FONCTION SEGMENTED_TRAJECTORY                   */
/* ------------------------------------------------------------- */
void handle_segmented_trajectory() {
    if (!domain1_pd || off_axisY_target == 0) {
        fprintf(stderr, "[TRAJECTORY] Offsets PDO invalides pour handle_segmented_trajectory()\n");
        current_state = ERROR;
        return;
    }

    if (!motion_started) {

        printf("[TRAJECTORY] Configuration de la trajectoire segmentée...\n");
        axisY_target_pos_um = MM_TO_UM(DIST_AVANCE_VIDE1 + DIST_EXTRU_LENTE + DIST_EXTRU_RAPIDE + DIST_AVANCE_VIDE2);
        printf("[TRAJECTORY] Cible : %.3f mm\n", UM_TO_MM(axisY_target_pos_um));

        // --- Écrire la cible dans le PDO ---
        printf("[TRAJECTORY] Écriture de la cible dans PDO...\n");
        EC_WRITE_S32(domain1_pd + off_axisY_target, axisY_target_pos_um);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(50000);

        // --- Envoyer la séquence de contrôle ---
        printf("[TRAJECTORY] Envoi de la séquence CiA-402...\n");
        if (send_controlword_sequence_via_pdo() != 0) {
            fprintf(stderr, "[TRAJECTORY] ERREUR : échec de l'envoi de la séquence de contrôle.\n");
            next_state = ERROR;
            return;
        }

        motion_started = true;
        motion_complete = true ;
        printf("[TRAJECTORY] Trajectoire démarrée. Cible = %.3f mm\n", UM_TO_MM(axisY_target_pos_um));
    }
}


/* ------------------------------------------------------------- */
/*                      FONCTION NETTOYAGE                       */
/* ------------------------------------------------------------- */
void cleanup_system() {
    printf("[CLEANUP] Arrêt de l'extrusion...\n");
    // Arrêt de l'extrusion
    user_velocity_steps_per_s = 0.0;
    for (int i = 0; i < 100; i++) {
        cyclic_task();
        usleep(1000);
    }

    // Désactivation du drive Y
    printf("[CLEANUP] Désactivation du drive Y...\n");
    if (domain1_pd && off_axisY_controlword != 0) {
        EC_WRITE_U16(domain1_pd + off_axisY_controlword, 0x0006);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
        usleep(100000);
    }

    // Fermeture des logs
    printf("[CLEANUP] Fermeture des logs...\n");
    if (log_extrusion_file) fclose(log_extrusion_file);
    if (log_position_file) fclose(log_position_file);

    // Libération du master
    printf("[CLEANUP] Libération du master EtherCAT...\n");
    ecrt_release_master(master);
    printf("[CLEANUP] Nettoyage terminé.\n");
}


/* -------------------------------------------------------------------------- */
/* THREAD TO COMPUTE EXTRUSION VELOCITY PROFILE BASED ON y_position_mm        */
/* -------------------------------------------------------------------------- */
void *supervisor_thread(void *arg)
{
    printf("\n[SUPERVISEUR] Démarré\n");
    printf("  Segments: %.1fmm vide | %.1fmm lent | %.1fmm rapide | %.1fmm vide\n",
           DIST_AVANCE_VIDE1, DIST_EXTRU_LENTE, DIST_EXTRU_RAPIDE, DIST_AVANCE_VIDE2);

    while (!stop && !motion_complete) {
        usleep(100000);
    }

    printf("[SUPERVISEUR] Arrêté\n");
    return NULL;
}


/* ------------------------------------------------------------- */
/*                      FONCTION PRINCIPALE                      */
/* ------------------------------------------------------------- */

int main(void) {
    signal(SIGINT, sigint_handler);
    printf("[MAIN] Démarrage du programme.\n");

    if (init_system() != 0) {  // <-- Vérifiez le retour
        fprintf(stderr, "[MAIN] Erreur : Initialisation échouée.\n");
        return 1;
    }

    current_state = IDLE;
    next_state = IDLE;
    motion_started = false;
    motion_complete = false;
    zero_reached = false;
    user_velocity_steps_per_s = 0.0;

    printf("[MAIN] État courant: IDLE.\n");

    pthread_t thread_sup;
    // Start supervisor thread
    if (pthread_create(&thread_sup, NULL, supervisor_thread, NULL) != 0) {
        perror("pthread_create");
        // non fatal
    }

    struct timespec wakeup;
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
        
    while (!stop) {
        printf("\n[MAIN] --- Boucle principale ---\n");
        printf("[MAIN] État courant: %d\n", current_state);

        switch (current_state) {
            case IDLE:
             printf("[MAIN] [IDLE] Vérification de la position Y (%.3f mm)...\n", y_position_mm);
                // Vérifie la position de l'axe Y
                if (fabs(y_position_mm) > 0.01) {
                    printf("[MAIN] [IDLE] Position Y > 0.01 mm → Transition vers MOVE_TO_ZERO.\n");
                    next_state = MOVE_TO_ZERO;
                } else {
                    printf("[MAIN] [IDLE] Position Y <= 0.01 mm → Transition vers SEGMENTED_TRAJECTORY.\n");
                    next_state = SEGMENTED_TRAJECTORY;
                }
                break;

            case MOVE_TO_ZERO:
                printf("[MAIN] [MOVE_TO_ZERO] Déplacement vers la position 0...\n");
                move_to_zero();
                next_state = zero_reached ? SEGMENTED_TRAJECTORY : (stop ? SHUTDOWN : MOVE_TO_ZERO);  
                break;

            case SEGMENTED_TRAJECTORY:
                printf("[MAIN] [SEGMENTED_TRAJECTORY] Exécution de la trajectoire...\n");
                handle_segmented_trajectory();
                next_state = motion_complete ? SHUTDOWN : SEGMENTED_TRAJECTORY;
                break;


            case ERROR:
                printf("[MAIN] [ERROR] État d'erreur → Transition vers SHUTDOWN.\n");
                next_state = SHUTDOWN;
                break;

            case SHUTDOWN:
                printf("[MAIN] [SHUTDOWN] Arrêt du système...\n");
                cleanup_system();
                stop = 1;
                break;

            default:
                next_state = ERROR;
                break;
        }
        printf("[MAIN] Prochain état: %d\n", next_state);

        // Exécution de la tâche cyclique (1ms)
        if (current_state != SHUTDOWN) {
            cyclic_task();
        }

        // Transition vers le prochain état
        current_state = next_state;


        // Gestion du temps réel (1ms)
        wakeup.tv_nsec += PERIOD_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) {
            wakeup.tv_nsec -= NSEC_PER_SEC;
            wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);

        // break loop if motion complete and SHUTDOWN requested
        if (motion_complete && current_state == SHUTDOWN) break;
    }

    // join supervisor
    stop = 1;
    pthread_join(thread_sup, NULL);

    printf("[MAIN] Programme terminé.\n");
    return 0;
}


