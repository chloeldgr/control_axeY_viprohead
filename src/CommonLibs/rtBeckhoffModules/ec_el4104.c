#include "ec_el4104.h"

#define MAX_OUTPUT_VALUE    10
#define MIN_OUTPUT_VALUE    0
#define DAC_RESOLUTION      65536
#define Beckhoff_EL4104     0x00000002, 0x10083052

static uint16_t index_channel[NB_EL4104_CHANNEL] = {0x7000,
                                                    0x7010,
                                                    0x7020,
                                                    0x7030};

static EL4104_STRUCT ec_el4104[MAX_NB_EL4104];


int EL4104_Config(  unsigned int id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain1)
{
    int i;
    if ( id > MAX_NB_EL4104)
        return -1;

    /* Initialisation des paramètres de la structure*/


    if (!(ec_el4104[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL4104)))
        return -1;

    for (i= 0; i < NB_EL4104_CHANNEL;i++)
    {
        ec_el4104[id].offset_output[i]= ecrt_slave_config_reg_pdo_entry(ec_el4104[id].sc,
                                                                 index_channel[i],
                                                                 0x01,
                                                                 pdomain1,
                                                                 NULL);
        if (ec_el4104[id].offset_output[i] < 0)
            return -1;

    }
    return 0;
}


int EL4104_Write(unsigned int id, uint8_t *pdomain_pd, double *output_value)
{
    int16_t dac_value;
    int i;
    if ( id > MAX_NB_EL4104)
        return -1;

    for (i = 0 ; i < NB_EL4104_CHANNEL ; i++)
    {
        // Vérification Nan a
        if ( isnan(output_value[i]))
            output_value[i] = 0;
        // Saturation
        if (output_value[i] > MAX_OUTPUT_VALUE)
            output_value[i] = MAX_OUTPUT_VALUE;
        if (output_value[i] < MIN_OUTPUT_VALUE)
            output_value[i] = MIN_OUTPUT_VALUE;

        //Conversion DAC de la tension de sortie puis envoi
        dac_value = (int16_t)(output_value[i]*(double)DAC_RESOLUTION/(20.0));
        EC_WRITE_U16(pdomain_pd + ec_el4104[id].offset_output[i],dac_value);
    }
    return 0;
}
