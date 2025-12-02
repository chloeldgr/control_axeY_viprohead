#include "ethercat_utility.h"
#include <signal.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <errno.h>

/* -------------------------------------------------------------------------- */
/*                              SIGNAL HANDLER                                */
/* -------------------------------------------------------------------------- */
void sigint_handler(int s) {
    stop = 1;
}

/* -------------------------------------------------------------------------- */
/*                  ETHERCAT STATE CHECKING HELPERS                           */
/* -------------------------------------------------------------------------- */
void check_domain_state(EtherCATContext ec_context, EtherCATState ec_state)
{
    ec_domain_state_t ds;
    ecrt_domain_state(ec_context.domain1, &ds);
    if (ds.working_counter != ec_state.domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != ec_state.domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }
    ec_state.domain1_state = ds;
}

void check_master_state(EtherCATContext ec_context, EtherCATState ec_state)
{
    ec_master_state_t ms;
    ecrt_master_state(ec_context.master, &ms);
    if (ms.slaves_responding != ec_state.master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != ec_state.master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != ec_state.master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }
    ec_state.master_state = ms;
}

void check_extruder_config_states(Extruder extruder_to_check)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(extruder_to_check.sc_extruder, &s);
    if (s.al_state != extruder_to_check.sc_extruder_state.al_state) {
        printf("Extruder: State 0x%02X.\n", s.al_state);
    }
    if (s.online != extruder_to_check.sc_extruder_state.online) {
        printf("Extruder: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != extruder_to_check.sc_extruder_state.operational) {
        printf("Extruder: %soperational.\n", s.operational ? "" : "Not ");
    }
    extruder_to_check.sc_extruder_state = s;

}

void check_axis_config_states(Axis axis_to_check)
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(axis_to_check.sc_axis, &s);

    if (s.al_state != axis_to_check.sc_axis_state.al_state) {
        printf("Axis Y: State 0x%02X.\n", s.al_state);
    }
    if (s.online != axis_to_check.sc_axis_state.online) {
        printf("Axis Y: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != axis_to_check.sc_axis_state.operational) {
        printf("Axis Y: %soperational.\n", s.operational ? "" : "Not ");
    }

    axis_to_check.sc_axis_state = s;
}

void ec_check_and_print_states(EtherCATContext ec_context, EtherCATState ec_state, Extruder extruder_to_check, Axis axis_to_check) {
    check_domain1_state(ec_context, ec_state);
    check_master_state(ec_context, ec_state);
    check_extruder_config_states(extruder_to_check);
    check_axis_config_states(axis_to_check);
    }


/* -------------------------------------------------------------------------- */
/*                          SDO READ/WRITE HELPERS                            */
/* -------------------------------------------------------------------------- */
int write_sdo(EtherCATContext ec_context, uint16_t slave, uint16_t index, uint8_t sub, const void *val, size_t size) {
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
        ret = ecrt_master_sdo_download(ec_context.master, slave, index, sub, (uint8_t *)val, size, &abort);
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

int read_sdo(EtherCATContext ec_context, uint16_t slave, uint16_t index, uint8_t sub, void *target, size_t size) {
    uint32_t abort;
    size_t result_size;
    int ret = ecrt_master_sdo_upload(ec_context.master, slave, index, sub, (uint8_t *)target, size, &result_size, &abort);
    if (ret < 0) {
        fprintf(stderr, "Erreur upload SDO 0x%04X:%02X (abort=0x%08X)\n", index, sub, abort);
    }
    return ret;
}



/* -------------------------------------------------------------------------- */
/*                     SINGLE AXIS POSITION CONTROL                    */
/* -------------------------------------------------------------------------- */
void ec_set_target_position(EtherCATContext ec_context, Axis axis_to_move, int32_t target_um) {
    if (axis_to_move.off_axis_target != 0) {
        EC_WRITE_S32(ec_context.domain1_pd + axis_to_move.off_axis_target, target_um);
        ecrt_domain_queue(ec_context.domain1);
        ecrt_master_send(ec_context.master);
        usleep(50000);
    }
}

bool is_position_reached(double current, double target) {
    return fabs(current - target) < EPSILON;
}

/* -------------------------------------------------------------------------- */
/*                  CONTROLWORD SEQUENCE (CIA-402) SINGLE AXIS                */
/* -------------------------------------------------------------------------- */
/* Activation sequence via PDO: send controlword by PDO (shutdown/switch-on/enable) */
int config_and_launch_single_axis(EtherCATContext ec_context, Axis axis_to_config) {
    if (!ec_context.domain1_pd || axis_to_config.off_axis_controlword == 0) {
        fprintf(stderr, "Controlword PDO offset invalide\n");
        return -1;
    }
    /* If target offset is available, write the target position into PDO before enabling */
    if (axis_to_config.off_axis_target != 0) {
        ec_set_target_position(ec_context, axis_to_config, axis_to_config.axis_target_pos_um);
    }
    uint16_t cw = 0x0006; // Shutdown
    ec_send_controlword(ec_context, axis_to_config, cw);

    cw = 0x0007; // Switch on
    ec_send_controlword(ec_context, axis_to_config, cw);

    cw = 0x000F; // Enable operation
    ec_send_controlword(ec_context, axis_to_config, cw);

    /* Start motion: set new setpoint + start (use the pattern que vous utilisiez précédemment)
    Here we set bit new setpoint + start bits equal to 0x009B */
    cw = 0x009B;
    ec_send_controlword(ec_context, axis_to_config,cw);

    printf("Controlword sequence via PDO sent\n");
    return 0;
}

/* -------------------------------------------------------------------------- */
/*                           SHUTDOWN SINGLE AXE                              */
/* -------------------------------------------------------------------------- */

void ec_shutdown_single_axis(EtherCATContext ec_context,
                            Axis axis_to_shutdown) {

    ec_send_controlword(ec_context, axis_to_shutdown, 0x0006); // Shutdown

}


/* -------------------------------------------------------------------------- */
/*                             ENVOIE CONTROL WORD DRIVE AXES                 */
/* -------------------------------------------------------------------------- */

void ec_send_controlword(EtherCATContext ec_context, 
                        Axis axis_to_send_cw, 
                        uint16_t controlword) {

    if (axis_to_send_cw.off_axis_controlword != 0) {
        EC_WRITE_U16(ec_context.domain1_pd + axis_to_send_cw.off_axis_controlword, controlword);
        ecrt_domain_queue(ec_context.domain1);
        ecrt_master_send(ec_context.master);
        usleep(100000);
    }

}





/* -------------------------------------------------------------------------- */
/*                              EXTRUDER CONTROL                              */
/* -------------------------------------------------------------------------- */
void ec_enable_extruder(EtherCATContext ec_context, Extruder extruder_to_enable, bool enable) {
    if (extruder_to_enable.off_extruder_control_byte != 0) {
        EC_WRITE_U8(ec_context.domain1_pd + extruder_to_enable.off_extruder_control_byte, enable ? 1 : 0);
    }
}

void ec_set_extruder_velocity(EtherCATContext ec_context, Extruder extruder_to_set_velocity, double velocity_steps_per_s) {
    if (extruder_to_set_velocity.off_extruder_velocity != 0) {
        uint16_t scaled_velocity = (uint16_t)(velocity_steps_per_s * FACTOR);
        EC_WRITE_U16(ec_context.domain1_pd + extruder_to_set_velocity.off_extruder_velocity, scaled_velocity);
    }
}



/* -------------------------------------------------------------------------- */
/*                       LOG POSITION AND EXTRUSION                           */
/* -------------------------------------------------------------------------- */
void log_position_and_extrusion(LogFiles log_files, double timestamp, double position_mm, double extrusion_steps_per_s) {
    if (log_files.log_extrusion_file) {
        fprintf(log_files.log_extrusion_file, "%f,%.3f,%.1f\n", timestamp, position_mm, extrusion_steps_per_s);
        fflush(log_files.log_extrusion_file);
    }
    printf("[EVENT] Extrusion=%.1f steps/s à Y=%.2f mm\n", extrusion_steps_per_s, position_mm);
}



/* -------------------------------------------------------------------------- */
/*                      DEMARRER SEQUENCE HOMING                              */
/* -------------------------------------------------------------------------- */
void start_homing_sequence(EtherCATContext ec_context, Axis axis_to_homing) {
    printf("=== Début du Homing ===\n");
    ec_send_controlword(ec_context, axis_to_homing, 0x0006); // Shutdown
    ec_send_controlword(ec_context, axis_to_homing, 0x0007); // Switch On
    ec_send_controlword(ec_context, axis_to_homing, 0x000F); // Enable Operation
    ec_set_target_position(ec_context, axis_to_homing, 0);
    ec_send_controlword(ec_context, axis_to_homing, 0x009B); // Start Motion
}

void homing_sequence(EtherCATContext ec_context, 
                    Axis axis_to_homing ) {

    if (!ec_context.domain1_pd || axis_to_homing.off_axis_target == 0 || axis_to_homing.off_axis_controlword == 0) {
        fprintf(stderr, "Offsets PDO invalides pour homing_sequence()\n");
        return;
    }

    start_homing_sequence(ec_context, axis_to_homing);

    // Boucle bloquante jusqu'à ce que la position réelle atteigne 0
    printf("Déplacement en cours...\n");

    int zero_reached_count = 0;
    int32_t cur_pos_um = 0;
    double cur_mm = 0.0;

    printf("=== Retour à la position 0 (Homing) ===\n");
    printf("Position actuelle: %.8f mm\n", cur_mm);

    while (!stop) {
        ecrt_master_receive(ec_context.master);
        ecrt_domain_process(ec_context.domain1);

        cur_pos_um = EC_READ_S32(ec_context.domain1_pd + axis_to_homing.off_axis_pos_actual);
        cur_mm = UM_TO_MM(cur_pos_um);
        uint16_t axis_status = EC_READ_U16(ec_context.domain1_pd + axis_to_homing.off_axis_statusword);

        printf("ZeroCount : %d ,Position actuelle: %.5f mm, Statusword: 0x%04X\n", zero_reached_count, cur_mm, axis_status);

        // Vérifier si la cible est atteinte ou si la position est proche de 0
        if (is_position_reached(cur_mm,0.0)) { // <-- condition ne fonctionne pas lorsqu'on lance le code et qu'on est déjà en 0mm
            zero_reached_count++;
        } else {
            zero_reached_count=0;
        }

        // Exiger 100 cycles consécutifs pour confirmer
        if (zero_reached_count >= 100) {
            printf("Position 0 atteinte.\n");
            cycle_mode = CYCLE_SEGMENTED_TRAJECTORY;
            break;
        }
        ecrt_domain_queue(ec_context.domain1);
        ecrt_master_send(ec_context.master);
        usleep(5000); // 5ms
    }

}



/* -------------------------------------------------------------------------- */
/*                          CYCLIC TASK RT (1 ms LOOP)                        */
/* -------------------------------------------------------------------------- */

void cyclic_task(EtherCATContext ec_context, EtherCATState ec_state, Extruder extruder_to_use, Axis axis_to_move, SystemState system_state, CycleMode cycle_mode, TrajectoryParams traj_segmented, LogFiles log_files) {
    // --- Receive PDOs, process domain ---
    ecrt_master_receive(ec_context.master);
    ecrt_domain_process(ec_context.domain1);

    if (counter == 0) {
        counter = 1000;
        ec_check_and_print_states(ec_context, ec_state, extruder_to_use, axis_to_move);
    }
    counter--;

    // --- Read Axis Y position if offset valid ---
    int32_t position_um = EC_READ_S32(ec_context.domain1_pd + axis_to_move.off_axis_pos_actual);
    system_state.y_position_mm = UM_TO_MM(position_um);
    // int32_t pos_um = 0;
    // double y = 0.0;
    // if (pdoOffsets.off_axisY_pos_actual != 0) {
    //     pos_um = EC_READ_S32(ecContext.domain1_pd + pdoOffsets.off_axisY_pos_actual);
    //     y = UM_TO_MM(pos_um);
    //     system_state.y_position_mm = y;
    // }

    // --- Read statusword Y ---
    uint16_t axis_status = EC_READ_U16(ec_context.domain1_pd + axis_to_move.off_axis_statusword);
    // uint16_t axis_status = 0;
    // if (pdoOffsets.off_axisY_statusword != 0) {
    //     axis_status = EC_READ_U16(ecContext.domain1_pd + pdoOffsets.off_axisY_statusword);
    // }


    // --- GESTION DES MODES ---
    
    // --- Mode HOMING ---
    if (cycle_mode == CYCLE_HOMING) {
        homing_sequence(ec_context, axis_to_move);
    }

    // --- Mode TRAJECTORY ---
    if (cycle_mode == CYCLE_SEGMENTED_TRAJECTORY && system_state.motion_started && !system_state.motion_complete) {
        static int etape = 0;
        static double last_extrusion = -9999.0;

        double seuil1 = traj_segmented.dist_avance_vide1;
        double seuil2 = traj_segmented.dist_avance_vide1 + traj_segmented.dist_extru_lente;
        double seuil3 = traj_segmented.dist_avance_vide1 + traj_segmented.dist_extru_lente + traj_segmented.dist_extru_rapide;

        // Démarrer la trajectoire uniquement si elle n'est pas déjà démarrée
        if (!system_state.motion_started) {
            printf("[INFO] Départ trajectoire principale\n");
            axis_to_move.axis_target_pos_um = MM_TO_UM(traj_segmented.dist_avance_vide1 + traj_segmented.dist_extru_lente + traj_segmented.dist_extru_rapide + trajParams.dist_avance_vide2);
            system_state.motion_started = true;
            config_and_launch_single_axis(ec_context, axis_to_move);
        }

        switch (etape) {
            case 0:
                if (system_state.y_position_mm >= seuil1) {
                    system_state.user_velocity_steps_per_s = traj_segmented.extru_lente ;
                    etape = 1;
                }
                break;
            case 1:
                if (system_state.y_position_mm >= seuil2) {
                    system_state.user_velocity_steps_per_s = traj_segmented.extru_rapide;
                    etape = 2;
                }
                break;
            case 2:
                if (system_state.y_position_mm >= seuil3) {
                    system_state.user_velocity_steps_per_s = 0.0;
                    etape = 3;
                }
                break;
            default:
                break;
        }

        if (fabs(system_state.user_velocity_steps_per_s - last_extrusion) > 0.1) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            double t = ts.tv_sec + ts.tv_nsec * 1e-9;
            log_position_and_extrusion(log_files, t,system_state.y_position_mm, system_state.user_velocity_steps_per_s);
            last_extrusion = system_state.user_velocity_steps_per_s;
        }

        if (system_state.y_position_mm >= UM_TO_MM(axis_to_move.axis_target_pos_um) - 0.01) {
            system_state.motion_complete = true;
            printf("\n[INFO] Fin de trajectoire à %.2f mm.\n", system_state.y_position_mm);
        }
    }

    // --- Mode ERROR ---
    if (cycle_mode == CYCLE_ERROR) {
        printf("[ERREUR] Arrêt d'urgence ! Code erreur: %u\n", system_state.error_code);
        cycle_mode = CYCLE_SHUTDOWN;
    }

    // --- Mode SHUTDOWN ---
    if (cycle_mode == CYCLE_SHUTDOWN) {
        system_state.user_velocity_steps_per_s = 0.0;
        ec_enable_extruder(ec_context, extruder_to_use, false);
        ec_set_extruder_velocity(ec_context, extruder_to_use, 0);
        if (axis_to_move.off_axis_controlword != 0) {
            EC_WRITE_U16(ec_context.domain1_pd + axis_to_move.off_axis_controlword, 0x0006); // Shutdown
        }
        stop = 1;
    }

    // --- LOG DE LA POSITION Y ---
    if (log_files.log_position_file && cycle_mode != CYCLE_ERROR) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double t = ts.tv_sec + ts.tv_nsec * 1e-9;
        fprintf(log_files.log_position_file, "%f,%.3f,%.1f\n", t, system_state.y_position_mm, system_state.user_velocity_steps_per_s);
    }

    // --- Write extrusion speed ---

    ec_set_extruder_velocity(ec_context, extruder_to_use, system_state.user_velocity_steps_per_s) ;
    ec_enable_extruder(ec_context, extruder_to_use, true) ;


    ecrt_domain_queue(ec_context.domain1);
    ecrt_master_send(ec_context.master);
}

