// code issu de test_3.c transformé en machine à état
//
/* Controle de l'axe Y (ED1F CoE drive) et de l'extrudeur (EL7031) via etherCAT en C pur
 * Fonctionnalités :
 *  - Initialisation EtherCAT complète
 *  - Séquence CiA-402 de démarrage du drive
 *  - Lancement d’une trajectoire en mm
 *  - Gestion de l’extrusion selon position réelle
 *  -> utilisation de la machine à état
 */

// --- Pour build aller dans le dossier build et faire : ---
// gcc -o control_segmented_traj_state_machine ../src/control_segmented_traj_state_machine.c -lrt -pthread -L/usr/local/etherlab/lib -lethercat
// --- Pour lancer le code: ---
// sudo LD_LIBRARY_PATH=/usr/local/etherlab/lib ./control_segmented_traj_state_machine

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
    ec_slave_config_t *sc_extruder;
    ec_slave_config_t *sc_y;
    uint8_t *domain1_pd;
} EtherCATContext;

// Master / domain / slave configs (globales)
static ec_master_state_t master_state = {};
static ec_domain_state_t domain1_state = {};
static ec_slave_config_state_t sc_extruder_state = {};
static ec_slave_config_state_t sc_y_state = {};

typedef struct {
    unsigned int off_extruder_velocity;
    unsigned int off_extruder_control_byte;
    unsigned int off_extruder_status_byte;
    unsigned int off_axisY_pos_actual;
    unsigned int off_axisY_statusword;
    unsigned int off_axisY_controlword;
    unsigned int off_axisY_target;
    int32_t axisY_target_pos_um;
} PDOOffsets;

typedef struct {
    double dist_avance_vide1;
    double dist_extru_lente;
    double dist_extru_rapide;
    double dist_avance_vide2;
    double axeY_vel_mm_s;
    double extru_lente;
    double extru_rapide;
} TrajectoryParams;

enum {
    CYCLE_IDLE = 0,
    CYCLE_HOMING,
    CYCLE_SEGMENTED_TRAJECTORY,
    CYCLE_ERROR,
    CYCLE_SHUTDOWN
} cyclic_mode = CYCLE_IDLE;
/* -------------------------------------------------------------------------- */
/*                              GLOBAL VARIABLES                              */
/* -------------------------------------------------------------------------- */
volatile sig_atomic_t stop = 0;
SystemState systemState = {0};
LogFiles logsFiles = {NULL, NULL};
EtherCATContext ecContext = {NULL, NULL, NULL, NULL, NULL};
PDOOffsets pdoOffsets = {0,0,0,0,0,0,0,0};
TrajectoryParams trajParams = {
    .dist_avance_vide1 = 10.0,
    .dist_extru_lente = 50.0,
    .dist_extru_rapide = 50.0,
    .dist_avance_vide2 = 10.0,
    .axeY_vel_mm_s = 7.0,
    .extru_lente = 186.5,
    .extru_rapide = 266.43,
};

/* -------------------------------------------------------------------------- */
/*                         TEST PARAMETERS / SIMULATION                       */
/* -------------------------------------------------------------------------- */
#define PERIOD_NS 1000000L  // 1 ms
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS) // iterations/s

#define FACTOR 16.3835      // factor steps/s --> PDO (extruder)

/* Conversion mm <--> um parameters */
#define MM_TO_UM(x) ((int32_t)(x)*1000.0)
#define UM_TO_MM(x) ((double)(x)/1000.0)

/* CiA-402 Statusword bits */
#define SW_TARGET_REACHED      0x0400

void sigint_handler(int s) { stop = 1; }

/* -------------------------------------------------------------------------- */
/*                           EtherCAT Master Config                           */
/* -------------------------------------------------------------------------- */

/* Vendor/Product pairs */
#define ED1F 0x0000aaaa, 0x00000005  // Drive for Y axis (slave 3)
#define Beckhoff_EK1100 0x00000002, 0x01c33052 // Bus Coupler
#define Beckhoff_EL7031 0x00000002, 0x1b773052 // Drive for Viprohead extruder

/* Bus positions (Master, position) */
#define BusCouplerPos    0, 4
#define AxisYPos         0, 3  // slave 3 = Axis Y (read position)
#define ExtruderPos      0, 5  // slave 5 = EL7031


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
    { AxisYPos, ED1F, 0x6064, 0x00, &pdoOffsets.off_axisY_pos_actual }, 
    { AxisYPos, ED1F, 0x6041, 0x00, &pdoOffsets.off_axisY_statusword }, 
    { AxisYPos, ED1F, 0x6040, 0x00, &pdoOffsets.off_axisY_controlword }, 
    { AxisYPos, ED1F, 0x607A, 0x00, &pdoOffsets.off_axisY_target}, 
    /* Extruder EL7031 */ 
    { ExtruderPos, Beckhoff_EL7031, 0x7010, 0x01, &pdoOffsets.off_extruder_control_byte }, // control byte 
    { ExtruderPos, Beckhoff_EL7031, 0x7010, 0x21, &pdoOffsets.off_extruder_velocity }, // velocity (16-bit in your mapping) 
    { ExtruderPos, Beckhoff_EL7031, 0x6010, 0x01, &pdoOffsets.off_extruder_status_byte }, // status byte 
    {} /* terminator */ 
    };

/* -------------------------------------------------------------------------- */
/*                  STATE CHECKING HELPERS                                    */
/* -------------------------------------------------------------------------- */
void check_domain1_state(void)
{
    ec_domain_state_t ds;
    ecrt_domain_state(ecContext.domain1, &ds);
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
    ecrt_master_state(ecContext.master, &ms);
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
    ecrt_slave_config_state(ecContext.sc_extruder, &s);
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


    ecrt_slave_config_state(ecContext.sc_y, &s);
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
static int write_sdo(uint16_t slave, uint16_t index, uint8_t sub, const void *val, size_t size) {
    uint32_t abort = 0;
    printf(" Écriture SDO esclave %u: 0x%04X:%02X... ", slave, index, sub);

    sigset_t newset, oldset;
    sigemptyset(&newset);
    sigaddset(&newset, SIGINT);
    if (pthread_sigmask(SIG_BLOCK, &newset, &oldset) != 0) {
        perror("pthread_sigmask");
    }
    int max_attempts = 5;
    int attempt = 0;
    int ret = -1;
    while (attempt < max_attempts) {
        ret = ecrt_master_sdo_download(ecContext.master, slave, index, sub, (uint8_t *)val, size, &abort);
        if (ret >= 0) {
            break;
        }
        if (errno == EINTR) {
            attempt++;
            fprintf(stderr, "SDO interrupted, retry %d/%d...\n", attempt, max_attempts);
            usleep(100000);
            continue;
        } else {
            fprintf(stderr, "Failed download SDO (abort=0x%08X) (ret=%d errno=%d)\n", abort, ret, errno);
            break;
        }
    }

    if (pthread_sigmask(SIG_SETMASK, &oldset, NULL) != 0) {
        perror("pthread_sigmask restore");
    }

    if (ret < 0) {
        return ret;
    }

    printf("OK\n");
    return ret;
}

static int read_sdo(uint16_t slave, uint16_t index, uint8_t sub, void *target, size_t size) {
    uint32_t abort;
    size_t result_size;
    int ret = ecrt_master_sdo_upload(ecContext.master, slave, index, sub, (uint8_t *)target, size, &result_size, &abort);
    if (ret < 0) {
        fprintf(stderr, "Erreur upload SDO 0x%04X:%02X (abort=0x%08X)\n", index, sub, abort);
    }
    return ret;
}


/* -------------------------------------------------------------------------- */
/*          CONFIGURATION AND LAUNCH Y-AXIS (via PDO controlword)             */
/* -------------------------------------------------------------------------- */
/* Activation sequence via PDO: send controlword by PDO (shutdown/switch-on/enable) */
int send_controlword_sequence_via_pdo() {
    if (!ecContext.domain1_pd || pdoOffsets.off_axisY_controlword == 0) {
        fprintf(stderr, "Controlword PDO offset invalide\n");
        return -1;
    }
    /* If target offset is available, write the target position into PDO before enabling */
    if (pdoOffsets.off_axisY_target != 0) {
        EC_WRITE_S32(ecContext.domain1_pd + pdoOffsets.off_axisY_target, pdoOffsets.axisY_target_pos_um);
        ecrt_domain_queue(ecContext.domain1);
        ecrt_master_send(ecContext.master);
        usleep(50000);
    }
    uint16_t cw = 0x0006; // Shutdown
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    cw = 0x0007; // Switch on
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    cw = 0x000F; // Enable operation
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    /* Start motion: set new setpoint + start (use the pattern que vous utilisiez précédemment)
    Here we set bit new setpoint + start bits equal to 0x009B */
    cw = 0x009B;
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    printf("Controlword sequence via PDO sent\n");
    return 0;
}

/* -------------------------------------------------------------------------- */
/*                          HOMING SEQUENCE                                   */
/* -------------------------------------------------------------------------- */
void homing_sequence() {
    if (!ecContext.domain1_pd || pdoOffsets.off_axisY_target == 0 || pdoOffsets.off_axisY_controlword == 0) {
        fprintf(stderr, "Offsets PDO invalides pour homing_sequence()\n");
        return;
    }

    int32_t cur_pos_um = EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisY_pos_actual);
    double cur_mm = UM_TO_MM(cur_pos_um);
    printf("=== Retour à la position 0 (Homing) ===\n");
    printf("Position actuelle: %.2f mm\n", cur_mm);

    // if (cur_mm <= 0.1) {
    //     printf("Déjà en position 0.\n");
    //     cyclic_mode = CYCLE_SEGMENTED_TRAJECTORY;
    //     return;
    // }

    // Séquence CiA-402 pour démarrer le mouvement vers zéro
    uint16_t cw = 0x0006; // Shutdown
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    cw = 0x0007; // Switch on
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    cw = 0x000F; // Enable operation
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    // Écrire target = 0
    EC_WRITE_S32(ecContext.domain1_pd + pdoOffsets.off_axisY_target, 0);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(50000);

    // Bit "start" pour position control
    cw = 0x009B;
    EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, cw);
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
    usleep(100000);

    // Boucle bloquante jusqu'à ce que la position réelle atteigne 0
    printf("Déplacement en cours...\n");
    int zero_reached_count = 0;
    while (!stop) {
        ecrt_master_receive(ecContext.master);
        ecrt_domain_process(ecContext.domain1);

        cur_pos_um = EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisY_pos_actual);
        cur_mm = UM_TO_MM(cur_pos_um);
        uint16_t axis_status = EC_READ_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_statusword);

        printf("ZeroCount : %d ,Position actuelle: %.2f mm, Statusword: 0x%04X\n", zero_reached_count, cur_mm, axis_status);

        // Vérifier si la cible est atteinte ou si la position est proche de 0
        if ((cur_mm <= 0.1)) { // <-- condition ne fonctionne pas lorsqu'on lance le code et qu'on est déjà en 0mm
            zero_reached_count++;
        } else {
            zero_reached_count=0;
        }

        // Exiger 50 cycles consécutifs pour confirmer
        if (zero_reached_count >= 100) {
            printf("Position 0 atteinte.\n");
            cyclic_mode = CYCLE_SEGMENTED_TRAJECTORY;
            break;
        }

        ecrt_domain_queue(ecContext.domain1);
        ecrt_master_send(ecContext.master);
        usleep(5000); // 5ms
    }
}

/* -------------------------------------------------------------------------- */
/*                          CYCLIC TASK RT (1 ms LOOP)                        */
/* -------------------------------------------------------------------------- */
static unsigned int counter = 0;
void cyclic_task() {
    // --- Receive PDOs, process domain ---
    ecrt_master_receive(ecContext.master);
    ecrt_domain_process(ecContext.domain1);

    if (counter == 0) {
        counter = 1000;
        check_domain1_state();
        check_master_state();
        check_slave_config_states();
    }
    counter--;

    // --- Read Axis Y position if offset valid ---
    int32_t pos_um = 0;
    double y = 0.0;
    if (pdoOffsets.off_axisY_pos_actual != 0) {
        pos_um = EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisY_pos_actual);
        y = UM_TO_MM(pos_um);
        systemState.y_position_mm = y;
    }

    // --- Read statusword Y ---
    uint16_t axis_status = 0;
    if (pdoOffsets.off_axisY_statusword != 0) {
        axis_status = EC_READ_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_statusword);
    }

    // --- Mode HOMING ---
    if (cyclic_mode == CYCLE_HOMING) {
        homing_sequence();
    }

    // --- Mode TRAJECTORY ---
    if (cyclic_mode == CYCLE_SEGMENTED_TRAJECTORY && systemState.motion_started && !systemState.motion_complete) {
        static int etape = 0;
        static double last_extrusion = -9999.0;

        double seuil1 = trajParams.dist_avance_vide1;
        double seuil2 = trajParams.dist_avance_vide1 + trajParams.dist_extru_lente;
        double seuil3 = trajParams.dist_avance_vide1 + trajParams.dist_extru_lente + trajParams.dist_extru_rapide;

        // Démarrer la trajectoire uniquement si elle n'est pas déjà démarrée
        if (!systemState.motion_started) {
            printf("[INFO] Départ trajectoire principale\n");
            pdoOffsets.axisY_target_pos_um = MM_TO_UM(trajParams.dist_avance_vide1 + trajParams.dist_extru_lente + trajParams.dist_extru_rapide + trajParams.dist_avance_vide2);
            systemState.motion_started = true;
            send_controlword_sequence_via_pdo();
        }

        switch (etape) {
            case 0:
                if (y >= seuil1) {
                    systemState.user_velocity_steps_per_s = trajParams.extru_lente ;
                    etape = 1;
                }
                break;
            case 1:
                if (y >= seuil2) {
                    systemState.user_velocity_steps_per_s = trajParams.extru_rapide;
                    etape = 2;
                }
                break;
            case 2:
                if (y >= seuil3) {
                    systemState.user_velocity_steps_per_s = 0.0;
                    etape = 3;
                }
                break;
            default:
                break;
        }

        if (fabs(systemState.user_velocity_steps_per_s - last_extrusion) > 0.1) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            double t = ts.tv_sec + ts.tv_nsec * 1e-9;

            if (logsFiles.log_extrusion_file) {
                fprintf(logsFiles.log_extrusion_file, "%f,%.3f,%.1f\n", t, systemState.y_position_mm, systemState.user_velocity_steps_per_s);
                fflush(logsFiles.log_extrusion_file);
            }
            printf("[EVENT] Extrusion=%.1f steps/s à Y=%.2f mm\n", systemState.user_velocity_steps_per_s, systemState.y_position_mm);
            last_extrusion = systemState.user_velocity_steps_per_s;
        }

        if (systemState.y_position_mm >= UM_TO_MM(pdoOffsets.axisY_target_pos_um) - 0.01) {
            systemState.motion_complete = true;
            printf("\n[INFO] Fin de trajectoire à %.2f mm.\n", systemState.y_position_mm);
        }
    }

    // --- Mode ERROR ---
    if (cyclic_mode == CYCLE_ERROR) {
        printf("[ERREUR] Arrêt d'urgence ! Code erreur: %u\n", systemState.error_code);
        cyclic_mode = CYCLE_SHUTDOWN;
    }

    // --- Mode SHUTDOWN ---
    if (cyclic_mode == CYCLE_SHUTDOWN) {
        systemState.user_velocity_steps_per_s = 0.0;
        if (pdoOffsets.off_extruder_control_byte != 0) {
            EC_WRITE_U8(ecContext.domain1_pd + pdoOffsets.off_extruder_control_byte, 0);
        }
        if (pdoOffsets.off_extruder_velocity != 0) {
            EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_extruder_velocity, 0);
        }
        if (pdoOffsets.off_axisY_controlword != 0) {
            EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, 0x0006); // Shutdown
        }
        stop = 1;
    }

    // --- LOG DE LA POSITION Y ---
    if (logsFiles.log_position_file && cyclic_mode != CYCLE_ERROR) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double t = ts.tv_sec + ts.tv_nsec * 1e-9;
        fprintf(logsFiles.log_position_file, "%f,%.3f,%.1f\n", t, systemState.y_position_mm, systemState.user_velocity_steps_per_s);
    }

    // --- Write extrusion speed ---
    uint16_t scaled = (uint16_t)(systemState.user_velocity_steps_per_s * FACTOR);
    if (pdoOffsets.off_extruder_control_byte != 0) {
        EC_WRITE_U8(ecContext.domain1_pd + pdoOffsets.off_extruder_control_byte, 1);
    }
    if (pdoOffsets.off_extruder_velocity != 0) {
        EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_extruder_velocity, scaled);
    }

    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
}


/* -------------------------------------------------------------------------- */
/* THREAD TO COMPUTE EXTRUSION VELOCITY PROFILE BASED ON y_position_mm        */
/* -------------------------------------------------------------------------- */
void *supervisor_thread(void *arg) {
    printf("\n[SUPERVISEUR] Démarré\n");
    printf("  Segments: %.1fmm vide | %.1fmm lent | %.1fmm rapide | %.1fmm vide\n",
           trajParams.dist_avance_vide1, trajParams.dist_extru_lente, trajParams.dist_extru_rapide, trajParams.dist_avance_vide2);
    while (!stop && !systemState.motion_complete && !systemState.system_error) {
        usleep(100000);
    }
    printf("[SUPERVISEUR] Arrêté\n");
    return NULL;
}


/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */
int main(void) {
    signal(SIGINT, sigint_handler);

    logsFiles.log_extrusion_file = fopen("extrusion_events.csv", "w");
    if (!logsFiles.log_extrusion_file) {
        perror("log extrusion");
        exit(1);
    }
    fprintf(logsFiles.log_extrusion_file, "timestamp_s,y_position_mm,extrusion_steps_s\n");

    logsFiles.log_position_file = fopen("position_log.csv", "w");
    if (!logsFiles.log_position_file) {
        perror("log position");
        exit(1);
    }
    fprintf(logsFiles.log_position_file, "timestamp_s,y_position_mm,extrusion_steps_s\n");

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║   EtherCAT Control - Axe Y + Extrudeur (Méthode Shell)     ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    /* --- Initialisation master EtherCAT --- */
    printf("=== Initialisation EtherCAT ===\n");
    ecContext.master = ecrt_request_master(0);
    if (!ecContext.master) {
        fprintf(stderr, "Erreur: impossible d'obtenir le master EtherCAT.\n");
        return 1;
    }

    ecContext.domain1 = ecrt_master_create_domain(ecContext.master);
    if (!ecContext.domain1) {
        fprintf(stderr, "Erreur: création du domaine échouée.\n");
        return 1;
    }

    /* --- Configuration slaves --- */
    printf("Configuring EK1100...\n");
    if (!ecrt_master_slave_config(ecContext.master, BusCouplerPos, Beckhoff_EK1100)) {
        fprintf(stderr, "Failed to configure EK1100.\n");
        return -1;
    }

    printf("Configuration ED1F (Axe Y)...\n");
    ecContext.sc_y = ecrt_master_slave_config(ecContext.master, AxisYPos, ED1F);
    if (!ecContext.sc_y) {
        fprintf(stderr, "Failed to configure ED1F.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(ecContext.sc_y, EC_END, slave_3_syncs)) {
        fprintf(stderr, "Failed to configure PDOs ED1F.\n");
        return -1;
    }

    printf("Configuration EL7031 (Extruder)...\n");
    ecContext.sc_extruder = ecrt_master_slave_config(ecContext.master, ExtruderPos, Beckhoff_EL7031);
    if (!ecContext.sc_extruder) {
        fprintf(stderr, "Failed to configure EL7031.\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(ecContext.sc_extruder, EC_END, el7031_syncs)) {
        fprintf(stderr, "Failed to configure PDOs EL7031.\n");
        return -1;
    }

    printf("Saving PDO...\n");
    if (ecrt_domain_reg_pdo_entry_list(ecContext.domain1, domain1_regs)) {
        fprintf(stderr, "Failed to save PDO!\n");
        return -1;
    }

    /* --- Configuration SDO avant activation --- */
    int32_t target_pos_um = MM_TO_UM(trajParams.dist_avance_vide1 + trajParams.dist_extru_lente + trajParams.dist_extru_rapide + trajParams.dist_avance_vide2);
    uint32_t vel_um_s = (uint32_t)(trajParams.axeY_vel_mm_s * 1000.0);
    int8_t mode_ppm = 1; // Profile Position Mode
    pdoOffsets.axisY_target_pos_um = target_pos_um;

    printf("\n=== Configuration SDO avant activation ===\n");
    if (write_sdo(3, 0x6060, 0x00, &mode_ppm, sizeof(mode_ppm)) < 0) {
        fprintf(stderr, "ERROR: Failed to configure mode (SDO)\n");
    }
    usleep(100000);
    if (write_sdo(3, 0x6081, 0x00, &vel_um_s, sizeof(vel_um_s)) < 0) {
        fprintf(stderr, "ERROR: Failed to configure target velocity (SDO)\n");
    }
    usleep(100000);
    if (write_sdo(3, 0x607A, 0x00, &target_pos_um, sizeof(target_pos_um)) < 0) {
        fprintf(stderr, "ERROR: Failed to configure target position (SDO)\n");
    }
    usleep(100000);
    printf("SDO initialisation done.\n");

    /* --- Activation du master --- */
    printf("Activating master...\n");
    if (ecrt_master_activate(ecContext.master)) {
        fprintf(stderr, "Failed activating master!\n");
        return -1;
    }

    ecContext.domain1_pd = ecrt_domain_data(ecContext.domain1);
    if (!ecContext.domain1_pd) {
        fprintf(stderr, "Failed returning domain data!\n");
        return -1;
    }

    printf("Offsets PDO:Y_pos=%u Y_status=%u Y_cw=%u Y_target=%u Ext_ctl=%u Ext_vel=%u\n",
           pdoOffsets.off_axisY_pos_actual, pdoOffsets.off_axisY_statusword, pdoOffsets.off_axisY_controlword, pdoOffsets.off_axisY_target,
           pdoOffsets.off_extruder_control_byte, pdoOffsets.off_extruder_velocity);

    printf("Exchange of initial PDO (3 secondes)...\n");
    struct timespec wakeup;
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    for (int i = 0; i < 3000; i++) {
        ecrt_master_receive(ecContext.master);
        ecrt_domain_process(ecContext.domain1);
        ecrt_domain_queue(ecContext.domain1);
        ecrt_master_send(ecContext.master);
        wakeup.tv_nsec += PERIOD_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) {
            wakeup.tv_nsec -= NSEC_PER_SEC;
            wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);
    }
    printf("Communication done!\n");

    systemState.motion_started = false;
    systemState.motion_complete = false;
    cyclic_mode = CYCLE_HOMING;

    pthread_t thread_sup;
    if (pthread_create(&thread_sup, NULL, supervisor_thread, NULL) != 0) {
        perror("pthread_create");
        fprintf(stderr, "Failed to create supervisor thread.\n");
    }

    printf("\n=== Déplacement vers 0mm (Homing) ===\n");
    printf("(CTRL+C to stop)\n\n");


    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    while (!stop) {
        cyclic_task();

        // Démarrer la trajectoire segmentée après le homing
        if (cyclic_mode == CYCLE_SEGMENTED_TRAJECTORY && !systemState.motion_started) {
            printf("[INFO] Départ trajectoire principale\n");
            pdoOffsets.axisY_target_pos_um = MM_TO_UM(trajParams.dist_avance_vide1 + trajParams.dist_extru_lente + trajParams.dist_extru_rapide + trajParams.dist_avance_vide2);
            systemState.motion_started = true;
            systemState.motion_complete = false;
            send_controlword_sequence_via_pdo();
        }

        wakeup.tv_nsec += PERIOD_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) {
            wakeup.tv_nsec -= NSEC_PER_SEC;
            wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);

        if (systemState.motion_complete || systemState.system_error) break;
    }

    printf("\n=== Stopping system ===\n");
    systemState.user_velocity_steps_per_s = 0.0;
    printf("Stop extrusion...\n");

    for (int i = 0; i < 100; i++) {
        cyclic_task();
        usleep(1000);
    }

    printf("Desactivation of drive Y...\n");
    if (ecContext.domain1_pd && pdoOffsets.off_axisY_controlword != 0) {
        EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_controlword, 0x0006);
        ecrt_domain_queue(ecContext.domain1);
        ecrt_master_send(ecContext.master);
        usleep(100000);
    }

    stop = 1;
    pthread_join(thread_sup, NULL);

    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║                      Programme done                        ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("Final position: %.2f mm\n\n", systemState.y_position_mm);

    ecrt_release_master(ecContext.master);
    if (logsFiles.log_extrusion_file) fclose(logsFiles.log_extrusion_file);
    if (logsFiles.log_position_file) fclose(logsFiles.log_position_file);

    return 0;
}