#include "ethercat_utility.h"



int init_ethercat(EtherCATContext *ec_context, int master_id) 
{
    ec_context->master = ecrt_request_master(0); // recup le handle du master (interface 0)
    if (!ec_context->master) {
        fprintf(stderr, "Failed to request master.\n");
        return -1;
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/*                  ETHERCAT STATE CHECKING HELPERS                           */
/* -------------------------------------------------------------------------- */
// void check_domain_state(EtherCATContext ec_context, EtherCATState ec_state)
// {
//     ec_domain_state_t ds;
//     ecrt_domain_state(ec_context.domain1, &ds);
//     if (ds.working_counter != ec_state.domain1_state.working_counter) {
//         printf("Domain1: WC %u.\n", ds.working_counter);
//     }
//     if (ds.wc_state != ec_state.domain1_state.wc_state) {
//         printf("Domain1: State %u.\n", ds.wc_state);
//     }
//     ec_state.domain1_state = ds;
// }

// void check_master_state(EtherCATContext ec_context, EtherCATState ec_state)
// {
//     ec_master_state_t ms;
//     ecrt_master_state(ec_context.master, &ms);
//     if (ms.slaves_responding != ec_state.master_state.slaves_responding) {
//         printf("%u slave(s).\n", ms.slaves_responding);
//     }
//     if (ms.al_states != ec_state.master_state.al_states) {
//         printf("AL states: 0x%02X.\n", ms.al_states);
//     }
//     if (ms.link_up != ec_state.master_state.link_up) {
//         printf("Link is %s.\n", ms.link_up ? "up" : "down");
//     }
//     ec_state.master_state = ms;
// }