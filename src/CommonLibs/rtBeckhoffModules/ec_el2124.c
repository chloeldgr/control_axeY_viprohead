#include "ec_el2124.h"

static EL2124_STRUCT ec_el2124[MAX_NB_EL2124];

#define Beckhoff_EL2124 0x00000002, 0x084c3052


int EL2124_Config(  uint8_t id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain1)
{
    if ( id > MAX_NB_EL2124)
        return -1;

    /* Initialisation des paramètres de la structure*/


    if (!(ec_el2124[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL2124)))
        return -1;


    //Output 1
    ec_el2124[id].offset_output= ecrt_slave_config_reg_pdo_entry(ec_el2124[id].sc,
                                                                 0x7000,
                                                                 0x01,
                                                                 pdomain1,
                                                                 NULL);
    if (ec_el2124[id].offset_output < 0)
        return -1;


    ec_el2124[id].bIsReady = 1;
    return 0;
}


int EL2124_Write(uint8_t id, uint8_t *pdomain_pd, uint8_t reg)
{
    EC_WRITE_U8(pdomain_pd+ec_el2124[id].offset_output, reg);
    return 0;
}
