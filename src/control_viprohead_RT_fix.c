// --- Pour build aller dans le dossier build et faire : ---
// gcc -o <nom_executable> ../src/<nom_fichier>.c -lrt -pthread -L/usr/local/etherlab/lib -lethercat
// --- Pour lancer le code: ---
// sudo LD_LIBRARY_PATH=/usr/local/etherlab/lib ./<nom_executable> <vitesse en steps/s ou rien> 


#define _POSIX_C_SOURCE 199309L // Active les fonctionnalités POSIX nécessaires (ex. clock_gettime, clock_nanosleep). Doit être défini avant <time.h>.

/****************************************************************************/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // pour atof()
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall()*/
#include <sched.h> /* sched_setscheduler() */
#include <malloc.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/
// Arret propre 
volatile sig_atomic_t stop = 0;
void sigint_handler(int signum) {
    stop = 1;
    printf("\nSignal reçu, arrêt en cours...\n");
}
/****************************************************************************/


/** Task period in ns. */
#define PERIOD_NS   (1000000) // periode de la boucle cyclique = 1 ms

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */
#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS) // nombre d'itération de la boucle par seconde
#define FACTOR 16.3835 // user speed scaling factor in steps/s

/****************************************************************************/

// EtherCAT --> Variables specifiques à la com avec le bus EtherCAT
    // _state ! structure pour récupérer les états (master, domain, slave..)
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL; // domaine des données process (zone memoire partagée entre master et application) 
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_motor = NULL; 
static ec_slave_config_state_t sc_motor_state = {};

// process data
static uint8_t *domain1_pd = NULL; // pointeur vers le buffer process data (après activation)

/****************************************************************************/

// Modules EtherCAT avex VendorID et Product Code
#define Beckhoff_EK1100 0x00000002, 0x01c33052
#define Beckhoff_EL7031 0x00000002, 0x1b773052

// Topologie du bus : ID Master et Slave Position
#define BusCouplerPos    0, 4 //0 si pas axes profilo
#define MotionSlavePos   0, 5 // 1 si pas axes profilo

/****************************************************************************/
// Structure des donnees PDO 
// Ces tableaux repr la structure logique des PDO et permettent au master de connaitre quelles entrees/sorties existent et leurs tailles
/* Master 0, Slave 1, "EL7031"
 * Vendor ID:       0x00000002
 * Product code:    0x1b773052
 * Revision number: 0x001a0000
 */

ec_pdo_entry_info_t el7031_pdo_entries[] = { // enumeration des entrees PDO correspondants à la doc du terminal
    {0x0000, 0x00, 1}, /* Gap */
    {0x7000, 0x02, 1}, /* Enable latch extern on positive edge */
    {0x7000, 0x03, 1}, /* Set counter */
    {0x7000, 0x04, 1}, /* Enable latch extern on negative edge */
    {0x0000, 0x00, 4}, /* Gap */
    {0x0000, 0x00, 8}, /* Gap */
    {0x7000, 0x11, 16}, /* Set counter value */
    // RxPDO (Master -> Slave)
    {0x7010, 0x01, 1}, /* Enable */
    {0x7010, 0x02, 1}, /* Reset */
    {0x7010, 0x03, 1}, /* Reduce torque */
    {0x7010, 0x21, 16}, /* Velocity */
    //
    {0x0000, 0x00, 5}, /* Gap */
    {0x0000, 0x00, 8}, /* Gap */
    {0x0000, 0x00, 1}, /* Gap */
    {0x6000, 0x02, 1}, /* Latch extern valid */
    {0x6000, 0x03, 1}, /* Set counter done */
    {0x6000, 0x04, 1}, /* Counter underflow */
    {0x6000, 0x05, 1}, /* Counter overflow */
    {0x0000, 0x00, 3}, /* Gap */
    {0x0000, 0x00, 4}, /* Gap */
    {0x6000, 0x0d, 1}, /* Status of extern latch */
    {0x6000, 0x0e, 1}, /* Sync error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x6000, 0x10, 1}, /* TxPDO Toggle */
    {0x6000, 0x11, 16}, /* Counter value */
    {0x6000, 0x12, 16}, /* Latch value */
    // TxPDO (Slave -> Master)
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
    {0x1a00, 13, el7031_pdo_entries + 13}, /* ENC TxPDO-Map Status compact */
    {0x1a03, 14, el7031_pdo_entries + 26}, /* STM TxPDO-Map Status */
};

ec_sync_info_t el7031_syncs[] = { // facon dont les PDO sont affectes aux SM 
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 3, el7031_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, el7031_pdos + 3, EC_WD_DISABLE},
    {0xff}
};

/****************************************************************************/
/* PDO Entry Offsets */
// --> Variables qui recevront les offsets (positions) dans domain1_pd pour chaque entree PDO enregistree
// Remplacement des variables uniques en octet entier : car plusieurs bits (enable/reset/error/...) partagent le même octet physique. C’est la raison des control_byte et status_byte.
// static unsigned int off_enable;
// static unsigned int off_reset;
static unsigned int off_velocity;
// static unsigned int off_ready;
// static unsigned int off_error;
// static unsigned int off_moving_pos;
// static unsigned int off_moving_neg;
static unsigned int off_control_byte;
static unsigned int off_status_byte ;

// PDO registration table
const static ec_pdo_entry_reg_t domain1_regs[] = { // domain1_regs[] registre fournit a ecrt_domain_reg_pdo_entry_list()
    // Pour chaque ligne on indique quel esclave, quel device, quel index/subindex a enregistrer et l'adresse(&) où le master ecrira l'offset associe
    {MotionSlavePos, Beckhoff_EL7031, 0x7010, 0x01, &off_control_byte}, // Octet de commande (contient enable/reset)
    {MotionSlavePos, Beckhoff_EL7031, 0x7010, 0x21, &off_velocity},    // Velocity
    {MotionSlavePos, Beckhoff_EL7031, 0x6010, 0x01, &off_status_byte},  // Octet de statut global
    {}
};

/****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds); // appelle ecrt_domain_state() pour recuperer l'etat du domain 

    if (ds.working_counter != domain1_state.working_counter) { // compare ds.working_counter à al valeur precedante
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) { // compare ds.wc_state à al valeur precedante
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms); // etat du maitre 
    
    // Affichage des changements
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

/****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s; 

    ecrt_slave_config_state(sc_motor, &s); // obtenir l'état de la config du EL7031 (online, al_state, operational)
    // Affichage des changements
    if (s.al_state != sc_motor_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_motor_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_motor_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_motor_state = s;
}

/****************************************************************************/
static double user_velocity_steps_per_s = 100.0; // example: user sets 100 steps/s
static unsigned int counter = 0; // utilise pour executer certaines actions toutes les FREQUENCY iterations (1 fois par seconde)

void cyclic_task() // Boucle temps reelle executee tous les 1 ms
{
    // receive process data
    ecrt_master_receive(master); // Recup les trames EtherCAT recentes (Rx)
    ecrt_domain_process(domain1); // traite les donnees recues pour peupler domain1_pd

    // check process data state
    check_domain1_state(); // affiche etats si changements

    if (counter) { 
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY; // Si counter est à 1, on remet counter a FREQUENCY + check master slaves
        // check for master state (optional)
        check_master_state();
        // check for slave configuration state(s) (optional)
        check_slave_config_states();

        // Show feedback : on lit l'octet "status" via EC_READ_U8(domain1_pd + off_status_byte)
        uint8_t status = EC_READ_U8(domain1_pd + off_status_byte);
        int ready       = (status >> 0) & 0x01; // Decodages des bits
        int ready2en    = (status >> 1) & 0x01;
        int warning     = (status >> 2) & 0x01;
        int error       = (status >> 3) & 0x01;
        int moving_pos  = (status >> 4) & 0x01;
        int moving_neg  = (status >> 5) & 0x01;

        printf("[Cycle] Ready:%d | Ready2en :%d | Warning: %d | Error:%d | Moving+:%d | Moving neg: %d | Vel:%.1f steps/s\n",
            ready, ready2en, warning , error, moving_pos, moving_neg, user_velocity_steps_per_s);

    }

    // --- Write process data --- 
        //Important : EC_READ_* et EC_WRITE_* sont des macros/inline fournis par ecrt.h pour lire/écrire sur domain*_pd en respectant le format (endian, tailles).
        // On compose l'octet de control. On met Enable (bit 0) a 1 toujours
    uint8_t control = 0;
    control |= (1 << 0); // bit 0 = Enable
    // control |= (1 << 1); // bit 1 = Reset (si tu veux activer le reset)
    EC_WRITE_U8(domain1_pd + off_control_byte, control); // On ecrit l'octet de controle

        // Compute scaled velocity
        // On transforme vitesse fourni par l'utilisateur en vitesse lue (facteur)
    
    uint16_t scaled_vel = (uint16_t)(user_velocity_steps_per_s * FACTOR);

    if(scaled_vel > 32767.0) scaled_vel = 32767 ;

    EC_WRITE_U16(domain1_pd + off_velocity, scaled_vel);

    ecrt_domain_queue(domain1); // met le domain en file pour etre envoye
    ecrt_master_send(master); // envoie des trames EtherCAT Tx
}


/****************************************************************************/

void stack_prefault(void) {
    // Ecrit sur la pile pour forcer l'allocation des pages memoires 
    // Appele avant la RTTask
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

int main(int argc, char **argv)
{   signal(SIGINT, sigint_handler);
    struct timespec wakeup_time ;
    int ret ;

    printf("Starting EtherCAT master for EL7031 stepper...\n") ;

    // --- Lecture de la vitesse utilisateur ---
    if (argc > 1) {
        user_velocity_steps_per_s = atof(argv[1]); // conversion en double via atof
        if (user_velocity_steps_per_s < 0) {
            printf("Warning: negative velocity not allowed, setting to 0.\n");
            user_velocity_steps_per_s = 0.0;
        }
    } else {
        printf("Usage: %s <vitesse_en_steps_par_seconde>\n", argv[0]);
        printf("Aucune vitesse spécifiée, utilisation de la valeur par défaut : %.1f steps/s.\n",
            user_velocity_steps_per_s);
    }
    // --- Creation master et domaine ---
    master = ecrt_request_master(0); // recup le handle du master (interface 0)
    if (!master) {
        fprintf(stderr, "Failed to request master.\n");
        return -1;
    }

    domain1 = ecrt_master_create_domain(master); // creation du domaine process pour regrouper les PDOs
    if (!domain1) {
        fprintf(stderr, "Failed to create domain.\n");
        return -1;
    }


    // --- Configure EK1100 (bus coupler) ---
    printf("Configuring EK1100...\n");
    ec_slave_config_t *sc;
    sc = ecrt_master_slave_config(master, BusCouplerPos, Beckhoff_EK1100); // ecrt_master_slave_config recup la structure de config des esclaves
    if (!sc) {
        fprintf(stderr, "Failed to configure EK1100.\n");
        return -1;
    }

    // --- Configure EL7031 ---
    printf("Configuring PDOs for EL7031...\n");
    sc_motor = ecrt_master_slave_config(master, MotionSlavePos, Beckhoff_EL7031);
    if (!sc_motor) {
        fprintf(stderr, "Failed to configure EL7031.\n");
        return -1;
    }

    // --- Enregistrement des PDOs dans le domaine ---
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) { // lecteur des entrees PDO declarees au domaine domain1
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
        // Lors de l'enregistrement, ecrt remplit off_control_byte, off_velocity et off_status_byte avec les offsets reels dans domain1_pd

    
    // --- Activation du master et recup du pointeur domain PD ---
    printf("Activating master...\n"); 
    if (ecrt_master_activate(master)) { // config communication et demarre l'echange PDO synchro
        fprintf(stderr, "Activation failed!\n");
        return -1;
    }

    if (!(domain1_pd = ecrt_domain_data(domain1))) { // retourne le pointeur sur la memoire partagee domain1_pd (après activation)
        fprintf(stderr, "Domain data failed!\n");
        return -1;
    }

    // --- Realtime scheduling setup ---
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // Définition du scheduling FIFO avec prio max dispo 

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    // --- Lock memory ---
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) { // mlockwall verrouille la memoire pour eviter le swapping 
        fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
    }

    stack_prefault(); // prealloue la pile pour eviter des pages faults


    // --- Boucle principale ---
    printf("Starting RT task (dt = %d ns)\n", PERIOD_NS);

        // On demarre la boucle dans le futur le temps que tout s'etablisse
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1;
    wakeup_time.tv_nsec = 0;

    while (!stop) {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL); // reveil la tache exactement au temps absolu wakeup_time
        if (ret) {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        cyclic_task(); // execution de la cyclic_task

        // Après chaque cyclic task, on incremente wakeup_tume de PERIOD_NS et on correige les depassements en secondes
        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    printf("Désactivation du moteur...\n");
    uint8_t control = 0;
    EC_WRITE_U8(domain1_pd + off_control_byte, control);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);

    printf("Arrêt du master EtherCAT.\n");
    ecrt_release_master(master);
    return 0;
}

