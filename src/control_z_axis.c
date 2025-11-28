/* control_z_axis.c 
- 
- 
- 
*/

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
/*                         TEST PARAMETERS / SIMULATION                       */
/* -------------------------------------------------------------------------- */
#define PERIOD_NS 1000000L  // 1 ms
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS) // iterations/s

#define FACTOR 16.3835      // factor steps/s --> PDO (extruder)

#define EPSILON 0.100 

/* Conversion mm <--> um parameters */
#define MM_TO_UM(x) ((int32_t)(x)*1000.0)
#define UM_TO_MM(x) ((double)(x)/1000.0)

/* CiA-402 Statusword bits */
#define SW_TARGET_REACHED      0x0400
volatile sig_atomic_t stop = 0;
void sigint_handler(int s) { stop = 1; }


// ==============================
// PDO OFFSETS AXIS Z
// ==============================
typedef struct {
    unsigned int off_axisZ_pos_actual;
    unsigned int off_axisZ_statusword;
    unsigned int off_axisZ_controlword;
    unsigned int off_axisZ_target;
    unsigned int off_axisZ_acceleration;
    unsigned int off_axisZ_deceleration;
    int32_t axisZ_target_pos_um;
} PDOOffsets;

typedef struct {
    ec_master_t *master;
    ec_domain_t *domain1;
    ec_slave_config_t *sc_z;
    uint8_t *domain1_pd;
} EtherCATContext;

// Master / domain / slave configs (globales)
static ec_master_state_t master_state = {};
static ec_domain_state_t domain1_state = {};
static ec_slave_config_state_t sc_z_state = {};

EtherCATContext ecContext = {NULL, NULL, NULL, NULL};
PDOOffsets pdoOffsets = {0,0,0,0,0,0,0};


/* Vendor/Product pairs */
#define ED1F 0x0000aaaa, 0x00000005  // Drive for Z axis (slave 0)
/* Bus positions (Master, position) */
#define AxisZPos         0, 0  // slave 0 = Axis Z (read position)

/* Master 0, Slave 0, "ED1F CoE Drive"
 * Vendor ID:       0x0000aaaa
 * Product code:    0x00000005
 * Revision number: 0x00010000
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x6072, 0x00, 16},
    {0x607a, 0x00, 32},
    {0x60b8, 0x00, 16},
    {0x60fe, 0x01, 32},
    {0x603f, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x6064, 0x00, 32},
    {0x60b9, 0x00, 16},
    {0x60ba, 0x00, 32},
    {0x60f4, 0x00, 32},
    {0x60fd, 0x00, 32},
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 6, slave_0_pdo_entries + 0},
    {0x1a00, 8, slave_0_pdo_entries + 6},
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


// Mapping PDO 
/* PDO registration table: register PDO entries we need into domain1 */ 
const static ec_pdo_entry_reg_t domain1_regs[] = { 
    /* Axis Y (ED1F drive at bus position 3) - read-only entries (TxPDO) */ 
    { AxisZPos, ED1F, 0x6064, 0x00, &pdoOffsets.off_axisZ_pos_actual }, 
    { AxisZPos, ED1F, 0x6041, 0x00, &pdoOffsets.off_axisZ_statusword }, 
    { AxisZPos, ED1F, 0x6040, 0x00, &pdoOffsets.off_axisZ_controlword }, 
    { AxisZPos, ED1F, 0x607A, 0x00, &pdoOffsets.off_axisZ_target}, 
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
    ecrt_slave_config_state(ecContext.sc_z, &s);
    if (s.al_state != sc_z_state.al_state) {
        printf("Extruder: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_z_state.online) {
        printf("Extruder: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_z_state.operational) {
        printf("Extruder: %soperational.\n", s.operational ? "" : "Not ");
    }
    sc_z_state = s;
}


/* -------------------------------------------------------------------------- */
/*                        FONCTIONS VERIF + AFFICHAGE ETATS                   */
/* -------------------------------------------------------------------------- */
static void ec_check_and_print_states(void) {
    check_domain1_state();
    check_master_state();
    check_slave_config_states();
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
/*                             ENVOIE CONTROL WORD                            */
/* -------------------------------------------------------------------------- */
static void ec_send_controlword(uint16_t controlword) {
    if (pdoOffsets.off_axisZ_controlword != 0) {
        EC_WRITE_U16(ecContext.domain1_pd + pdoOffsets.off_axisZ_controlword, controlword);
        ecrt_domain_queue(ecContext.domain1);
        ecrt_master_send(ecContext.master);
        usleep(100000);
    }
}

/* -------------------------------------------------------------------------- */
/*                             DEFINIR POSITION CIBLE                         */
/* -------------------------------------------------------------------------- */
static void ec_set_target_position(int32_t target_um) {
    if (pdoOffsets.off_axisZ_target != 0) {
        EC_WRITE_S32(ecContext.domain1_pd + pdoOffsets.off_axisZ_target, target_um);
        ecrt_domain_queue(ecContext.domain1);
        ecrt_master_send(ecContext.master);
        usleep(50000);
    }
}


/* -------------------------------------------------------------------------- */
/*                            EGALITE AVEC TOLERANCE                          */
/* -------------------------------------------------------------------------- */
static bool is_position_reached(double current, double target) {
    return fabs(current - target) < EPSILON;
}



/* -------------------------------------------------------------------------- */
/*          CONFIGURATION AND LAUNCH Z-AXIS (via PDO controlword)             */
/* -------------------------------------------------------------------------- */
/* Activation sequence via PDO: send controlword bZ PDO (shutdown/switch-on/enable) */
int send_controlword_sequence_via_pdo() {
    if (!ecContext.domain1_pd || pdoOffsets.off_axisZ_controlword == 0) {
        fprintf(stderr, "Controlword PDO offset invalide\n");
        return -1;
    }
    /* If target offset is available, write the target position into PDO before enabling */
    if (pdoOffsets.off_axisZ_target != 0) {
        ec_set_target_position(pdoOffsets.axisZ_target_pos_um);
    }
    uint16_t cw = 0x0006; // Shutdown
    ec_send_controlword(cw);

    cw = 0x0007; // Switch on
    ec_send_controlword(cw);

    cw = 0x000F; // Enable operation
    ec_send_controlword(cw);

    // Activation du frein (exemple : bit 0 pour O1)
    uint16_t brake_mask = 0x0001; // Bit 0 pour O1 (à adapter selon la doc)
    uint16_t brake_enable = 0x0001; // Activer le contrôle du frein
    // Écrire le masque pour activer le contrôle du frein (subindex 2)
    if (write_sdo(0, 0x60FE, 0x02, &brake_enable, sizeof(brake_enable)) < 0) {
        fprintf(stderr, "WARNING: Failed to enable brake control (SDO)\n");
    }
    usleep(100000);
    // Activer le frein (subindex 1)
    if (write_sdo(0, 0x60FE, 0x01, &brake_mask, sizeof(brake_mask)) < 0) {
        fprintf(stderr, "WARNING: Failed to set brake (SDO)\n");
    }
    usleep(100000);


    /* Start motion: set new setpoint + start (use the pattern que vous utilisiez précédemment)
    Here we set bit new setpoint + start bits equal to 0x009B */
    cw = 0x009B;
    ec_send_controlword(cw);

    // Après avoir activé le mode opérationnel, relâcher le frein
    brake_mask = 0x0000; // Désactiver le frein
    if (write_sdo(0, 0x60FE, 0x01, &brake_mask, sizeof(brake_mask)) < 0) {
        fprintf(stderr, "WARNING: Failed to release brake (SDO)\n");
    }
    usleep(100000);

    printf("Controlword sequence via PDO sent\n");
    return 0;
}

/* -------------------------------------------------------------------------- */
/*                              ARRET AXE Z                                   */
/* -------------------------------------------------------------------------- */

static void ec_shutdown_axis_Z(void) {
    ec_send_controlword(0x0006); // Shutdown
}


/* -------------------------------------------------------------------------- */
/*                      VARIABLES DE COMMANDE / ETAT                          */
/* -------------------------------------------------------------------------- */
static double position_z_target_mm = 0.0;     // mm, valeur de consigne (modifiable)
static double velocity_z_mm_s = 1.0;          // mm/s, vitesse souhaitée
static bool z_motion_started = false;
static bool z_motion_complete = false;


/* -------------------------------------------------------------------------- */
/*                          CYCLIC TASK RT (1 ms LOOP)                        */
/* -------------------------------------------------------------------------- */
static unsigned int counter = 0;

void cyclic_task() {
    // Receive + process
    ecrt_master_receive(ecContext.master);
    ecrt_domain_process(ecContext.domain1);

    // Periodic state printing (toutes les 1000 itérations)
    if (counter == 0) {
        counter = 1000;
        ec_check_and_print_states();
    }
    counter--;

    // Read Axis Z position & status if available
    int32_t pos_um = 0;
    double pos_mm = 0.0;
    uint16_t status = 0;

    if (pdoOffsets.off_axisZ_pos_actual != 0) {
        pos_um = EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisZ_pos_actual);
        pos_mm = UM_TO_MM(pos_um);
    }
    if (pdoOffsets.off_axisZ_statusword != 0) {
        status = EC_READ_U16(ecContext.domain1_pd + pdoOffsets.off_axisZ_statusword);
    }

    // Lecture des erreurs du drive
    uint16_t error_code = 0;
    if (read_sdo(0, 0x603F, 0x00, &error_code, sizeof(error_code)) < 0) {
        fprintf(stderr, "Failed to read error code.\n");
    } else if (error_code != 0) {
        printf("\n[ERROR] Drive error code: 0x%04X\n", error_code);
        // Optionnel : arrêter le mouvement en cas d'erreur
        // ec_shutdown_axis_Z();
    }

    // Si trajectoire demandée mais pas encore démarrée : écrire cible + envoyer sequence
    if (!z_motion_started) {
        pdoOffsets.axisZ_target_pos_um = MM_TO_UM(position_z_target_mm);
        // Écrire la cible (PDO)
        ec_set_target_position(pdoOffsets.axisZ_target_pos_um);

        // Ecrire la vitesse profil via SDO 0x6081 (profile velocity, units dépendant du drive)
        // Ici on convertit mm/s -> um/s (valeur en entier 32 bits)
        int32_t vel_um_s = (int32_t) (velocity_z_mm_s * 1000.0);
        // On tente d'écrire via PDO si mappé, sinon ne rien faire (SDO déjà écrit avant activation)
        // (pour sécurité on réécrit via PDO si 0x60fe est mappé dans ton PDO map)
        // Lancer la séquence de contrôle (shutdown->switch on->enable->start)
        send_controlword_sequence_via_pdo();
        z_motion_started = true;
        z_motion_complete = false;
        printf("[CMD] Move Z -> %.3f mm at %.3f mm/s (vel_um_s=%d)\n", position_z_target_mm, velocity_z_mm_s, vel_um_s);
    }
    
    // Détection d'arrivée
    bool reached = (status & SW_TARGET_REACHED) || is_position_reached(pos_mm, position_z_target_mm);
    if(reached && !z_motion_complete && z_motion_started){
        z_motion_complete = true;
        printf("\n[INFO] Z target reached: %.3f mm\n", pos_mm);
    }

    printf("PDO values - Pos: %d, Status: 0x%04X, Controlword: 0x%04X, Target: %d\n",
       EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisZ_pos_actual),
       EC_READ_U16(ecContext.domain1_pd + pdoOffsets.off_axisZ_statusword),
       EC_READ_U16(ecContext.domain1_pd + pdoOffsets.off_axisZ_controlword),
       EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisZ_target));

    // Optionnel : si terminé, on peut envoyer shutdown partiel (ou laisser en enabled)
    // Ici on ne change rien, on laisse l'axe en état opérationnel.

    // Print line (carriage return to overwrite)
    printf("Z pos: %.3f mm  target: %.3f mm  started: %d complete: %d\r", pos_mm, position_z_target_mm, z_motion_started, z_motion_complete);
    fflush(stdout);

    // Queue + send
    ecrt_domain_queue(ecContext.domain1);
    ecrt_master_send(ecContext.master);
}


/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */
int main(void) {
    signal(SIGINT, sigint_handler);

    printf("=== Initialisation EtherCAT (Axis Z only) ===\n");

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

    // Configure the ED1F slave at bus position AxisZPos
    printf("Configuration ED1F (Axe Z)...\n");
    ecContext.sc_z = ecrt_master_slave_config(ecContext.master, AxisZPos, ED1F);
    if (!ecContext.sc_z) {
        fprintf(stderr, "Failed to configure ED1F (Z).\n");
        return -1;
    }
    if (ecrt_slave_config_pdos(ecContext.sc_z, EC_END, slave_0_syncs)) {
        fprintf(stderr, "Failed to configure PDOs ED1F (Z).\n");
        return -1;
    }

    printf("Saving PDO...\n");
    if (ecrt_domain_reg_pdo_entry_list(ecContext.domain1, domain1_regs)) {
        fprintf(stderr, "Failed to save PDO!\n");
        return -1;
    }

    if (!pdoOffsets.off_axisZ_controlword || !pdoOffsets.off_axisZ_target || !pdoOffsets.off_axisZ_statusword || !pdoOffsets.off_axisZ_pos_actual) {
        fprintf(stderr, "ERROR: Failed PDO mapping for Axis Z. Offsets: cw=%u tgt=%u st=%u pos=%u\n",
                pdoOffsets.off_axisZ_controlword, pdoOffsets.off_axisZ_target, pdoOffsets.off_axisZ_statusword, pdoOffsets.off_axisZ_pos_actual);
        return -1;
    }

    /* --- Configuration SDO initial (profile velocity) avant activation --- */
    // Convert desired velocity (mm/s) -> um/s for SDO 0x6081 (Profile velocity)
    uint32_t vel_um_s = (uint32_t)(velocity_z_mm_s * 1000.0);
    int8_t mode_ppm = 1; // Profile Position Mode (1)
    int32_t target_pos_um = MM_TO_UM(position_z_target_mm);
    uint32_t acceleration = 10000; // Exemple : 5000 um/s²

    printf("\n=== Configuration SDO avant activation ===\n");
    if (write_sdo(0 /* slave index */, 0x6060, 0x00, &mode_ppm, sizeof(mode_ppm)) < 0) {
        fprintf(stderr, "WARNING: Failed to set mode of operation (SDO)\n");
    }
    usleep(100000);
    if (write_sdo(0 /* slave index */, 0x6081, 0x00, &vel_um_s, sizeof(vel_um_s)) < 0) {
        fprintf(stderr, "WARNING: Failed to set profile velocity (SDO)\n");
    }
    usleep(100000);
    if (write_sdo(0, 0x6083, 0x00, &acceleration, sizeof(acceleration)) < 0) {
        fprintf(stderr, "WARNING: Failed to set profile acceleration (SDO)\n");
    }
    usleep(100000);
    if (write_sdo(0, 0x6084, 0x00, &acceleration, sizeof(acceleration)) < 0) {
        fprintf(stderr, "WARNING: Failed to set profile deceleration (SDO)\n");
    }
    usleep(100000);
    if (write_sdo(0 /* slave index */, 0x607A, 0x00, &target_pos_um, sizeof(target_pos_um)) < 0) {
        fprintf(stderr, "WARNING: Failed to set initial target position (SDO)\n");
    }
    usleep(100000);

    /* --- Activation du master --- */
    printf("Activating master...\n");
    if (ecrt_master_activate(ecContext.master)) {
        fprintf(stderr, "Failed activating master!\n");
        return -1;
    }
    usleep(100000);

    ecContext.domain1_pd = ecrt_domain_data(ecContext.domain1);
    if (!ecContext.domain1_pd) {
        fprintf(stderr, "Failed returning domain data!\n");
        return -1;
    }

    printf("Offsets PDO: Z_pos=%u Z_status=%u Z_cw=%u Z_target=%u\n",
           pdoOffsets.off_axisZ_pos_actual, pdoOffsets.off_axisZ_statusword, pdoOffsets.off_axisZ_controlword, pdoOffsets.off_axisZ_target);

    /* --- Echange PDO initial pour stabiliser (1 seconde) --- */
    struct timespec wakeup;
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    for (int i = 0; i < 1000; i++) {
        cyclic_task();
        wakeup.tv_nsec += PERIOD_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) { wakeup.tv_nsec -= NSEC_PER_SEC; wakeup.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);
    }
    printf("\nCommunication initiale faite.\n");

    /* --- Initialisation des variables de commande --- */
    z_motion_started = false;
    z_motion_complete = false;

    /* --- Valeurs de test (modifiable) --- */
    position_z_target_mm = 10.0;   // cible en mm (ex : 50 mm)
    velocity_z_mm_s = 1.0;         // vitesse en mm/s (ex : 5 mm/s)
    printf("=== Commande de test: Z -> %.3f mm @ %.3f mm/s ===\n", position_z_target_mm, velocity_z_mm_s);

    /* --- Boucle principale temps réel --- */
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    while (!stop) {
        cyclic_task();

        wakeup.tv_nsec += PERIOD_NS;
        while (wakeup.tv_nsec >= NSEC_PER_SEC) {
            wakeup.tv_nsec -= NSEC_PER_SEC;
            wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);

        if (z_motion_complete) {
            printf("[MAIN] Motion complete, stopping loop.\n");
            break;
        }
    }

    printf("\n=== Stopping system ===\n");
    ec_shutdown_axis_Z();

    ecrt_release_master(ecContext.master);
    printf("Terminé.\n");
    return 0;
}