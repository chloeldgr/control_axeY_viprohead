#include "ec_el3104.h"

static EL3104_STRUCT ec_el3104[MAX_NB_EL3104];

#define Beckhoff_EL3104 0x00000002, 0x0c203052

static uint16_t index_channel[NB_CHANNEL_EL3104] = {   0x6000,
                                                0x6010,
                                                0x6020,
                                                0x6030};

int EL3104_Config(  unsigned int id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain1)
{
    int i;
    if ( id > MAX_NB_EL3104)
        return -1;



    /* Initialisation des param?tres de la structure*/
    if (!(ec_el3104[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL3104)))
        return -1;


    for (i = 0; i < NB_CHANNEL_EL3104 ;  i++)
    {
//        ec_el3104[id].input[i].offset_status= ecrt_slave_config_reg_pdo_entry(  ec_el3104[id].sc,
//                                                                                index_channel[i],
//                                                                                0x0f,
//                                                                                pdomain1,
//                                                                                NULL);
//        if (ec_el3104[id].input[i].offset_status < 0)
//            return -1;

        ec_el3104[id].input[i].offset_value_in = ecrt_slave_config_reg_pdo_entry(   ec_el3104[id].sc,
                                                                                    index_channel[i],
                                                                                    0x11,
                                                                                    pdomain1,
                                                                                    NULL);
        if (ec_el3104[id].input[i].offset_value_in < 0)
            return -1;


        ec_el3104[id].input[i].Toggle       = 0;
        ec_el3104[id].input[i].State        = 0;
        ec_el3104[id].input[i].SyncError    = 0;
        ec_el3104[id].input[i].Error        = 0;
        ec_el3104[id].input[i].Limit2       = 0;
        ec_el3104[id].input[i].Limit21      = 0;
        ec_el3104[id].input[i].Limit1       = 0;
        ec_el3104[id].input[i].Limit11      = 0;
        ec_el3104[id].input[i].Overrange    = 0;
        ec_el3104[id].input[i].Underrange   = 0;

        ec_el3104[id].input[i].value        = 0;
        ec_el3104[id].input[i].adc_resolution = 65536;
        ec_el3104[id].input[i].max_input_voltage = 10;
        ec_el3104[id].input[i].min_input_voltage = -10;
    }
    ec_el3104[id].bIsReady     = 0;

    return 0;
}


/**
 * @brief EL3104_Read
 * @param id
 * @param pdomain_pd
 * @param value
 * @return
 */
int EL3104_Read(uint8_t id,
                uint8_t *pdomain_pd)
{
    int i;
    if (id > MAX_NB_EL3104)
        return -1;

    for (i = 0; i < NB_CHANNEL_EL3104 ; i++)
        ec_el3104[id].input[i].value = EC_READ_S16(pdomain_pd+ec_el3104[id].input[i].offset_value_in);


    return 0;
}

/**
 * @brief EL3104_GetValue
 * @param id
 * @param id_channel
 * @param value
 * @return
 */
int  EL3104_GetValue(  uint8_t id,
                        uint8_t id_channel,
                        double *value)
{

    if ( id > MAX_NB_EL3104)
        return -1;
    if ( id_channel > NB_CHANNEL_EL3104)
        return -1;
    *value = (double)ec_el3104[id].input[id_channel].value*(ec_el3104[id].input[id_channel].max_input_voltage-ec_el3104[id].input[id_channel].min_input_voltage)/(double)ec_el3104[id].input[id_channel].adc_resolution;
    return 0;

}

