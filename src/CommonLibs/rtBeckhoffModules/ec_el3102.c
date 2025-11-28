#include "ec_el3102.h"

static EL3102_STRUCT ec_el3102[MAX_NB_EL3102];

#define Beckhoff_EL3102 0x00000002, 0x0c1e3052


int config_el3102(  unsigned int id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain1)
{
    int i;
    if ( id > MAX_NB_EL3102)
        return -1;



    /* Initialisation des paramètres de la structure*/
    if (!(ec_el3102[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL3102)))
        return -1;


    for (i = 0; i < NB_CHANNEL ;  i++)
    {
        ec_el3102[id].input[i].offset_status= ecrt_slave_config_reg_pdo_entry(  ec_el3102[id].sc,
                                                                                index_channel[i],
                                                                                0x01,
                                                                                pdomain1,
                                                                                NULL);
        if (ec_el3102[id].input[i].offset_status < 0)
            return -1;

        ec_el3102[id].input[i].offset_value_in = ecrt_slave_config_reg_pdo_entry(   ec_el3102[id].sc,
                                                                                    index_channel[i],
                                                                                    0x02,
                                                                                    pdomain1,
                                                                                    NULL);
        if (ec_el3102[id].input[i].offset_value_in < 0)
            return -1;


        ec_el3102[id].input[i].Toggle       = 0;
        ec_el3102[id].input[i].State        = 0;
        ec_el3102[id].input[i].SyncError    = 0;
        ec_el3102[id].input[i].Error        = 0;
        ec_el3102[id].input[i].Limit2       = 0;
        ec_el3102[id].input[i].Limit21      = 0;
        ec_el3102[id].input[i].Limit1       = 0;
        ec_el3102[id].input[i].Limit11      = 0;
        ec_el3102[id].input[i].Overrange    = 0;
        ec_el3102[id].input[i].Underrange   = 0;
    
        ec_el3102[id].input[i].value        = 0;
    }
    ec_el3102[id].bIsReady     = 0;
    
    return 0;
}

void read_reg_status_el3102(uint8_t id,uint8_t *pdomain_pd)
{
    uint16_t status;
    int i;
    for (i = 0 ; i < NB_CHANNEL ; i++)
    {
        status = EC_READ_U16(pdomain_pd+ec_el3102[id].input[i].offset_status);
        ec_el3102[id].input[i].Toggle          = status>>15 & 1 ;
        ec_el3102[id].input[i].State           = status>>14 & 1;
        ec_el3102[id].input[i].SyncError       = status>>13 & 1;
        ec_el3102[id].input[i].Error           = status>>6  & 1;
        ec_el3102[id].input[i].Limit2          = status>>5  & 1;
        ec_el3102[id].input[i].Limit21         = status>>4  & 1;
        ec_el3102[id].input[i].Limit1          = status>>3  & 1;
        ec_el3102[id].input[i].Limit11         = status>>2  & 1;
        ec_el3102[id].input[i].Overrange       = status>>1  & 1;
        ec_el3102[id].input[i].Underrange      = status>>0  & 1;
    }
}


int Get_EL3102_Status(uint8_t id)
{
    return ec_el3102[id].bIsReady;
}


/**
 * @brief read_el3102
 * @param id
 * @param pdomain_pd
 * @param id_channel
 * @param value
 * @return code d'erreur
 */
int read_el3102(uint8_t id,
                uint8_t *pdomain_pd,
                int id_channel,
                double *value)
{
    int i;
    if (id > MAX_NB_EL3102)
        return -1;
    if ( id_channel > NB_CHANNEL)
        return -2;

    for (i = 0; i < NB_CHANNEL ; i++)
        ec_el3102[id].input[i].value = EC_READ_S16(pdomain_pd+ec_el3102[id].input[i].offset_value_in);

    *value = (double)ec_el3102[id].input[id_channel].value*(max_input_voltage-min_input_voltage)/(double)adc_resolution;

    return 0;
}

