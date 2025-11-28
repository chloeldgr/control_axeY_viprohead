#include "ec_el4132.h"

#define MAX_NB_EL4132 256
#define NB_4132_OUTPUT 2
#define MAX_OUTPUT_VALUE 10
#define MIN_OUTPUT_VALUE -10
#define DAC_RESOLUTION 65536
#define Beckhoff_EL4132 0x00000002, 0x10243052


static EL4132_STRUCT ec_el4132[MAX_NB_EL4132];


int EL4132_Config(  unsigned int id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain1)
{
    if ( id > MAX_NB_EL4132)
        return -1;

    /* Initialisation des paramètres de la structure*/


    if (!(ec_el4132[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL4132)))
        return -1;

    ec_el4132[id].offset_output[0]= ecrt_slave_config_reg_pdo_entry(ec_el4132[id].sc,
                                                                 0x3001,
                                                                 0x01,
                                                                 pdomain1,
                                                                 NULL);
    if (ec_el4132[id].offset_output[0] < 0)
        return -1;

    ec_el4132[id].offset_output[1] = ecrt_slave_config_reg_pdo_entry(ec_el4132[id].sc,
                                                                    0x3002,
                                                                    0x01,
                                                                    pdomain1,
                                                                    NULL);
    if (ec_el4132[id].offset_output[1] < 0)
        return -1;

    return 0;
}


int EL4132_Write(unsigned int id, uint8_t *pdomain_pd, double *output_value)
{
    int16_t dac_value;
    int i;
    if ( id > MAX_NB_EL4132)
        return -1;

    for (i = 0 ; i < NB_4132_OUTPUT ; i++)
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
        dac_value = (int16_t)(output_value[i]*(double)DAC_RESOLUTION/20.0);

        EC_WRITE_S16(pdomain_pd + ec_el4132[id].offset_output[i],dac_value);
    }
    return 0;
}
