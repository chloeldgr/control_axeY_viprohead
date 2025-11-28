#include "ec_el4134.h"

#define MAX_NB_EL4134 256
#define NB_OUTPUT 4
#define MAX_OUTPUT_VALUE 10.0
#define MIN_OUTPUT_VALUE -10.0
#define DAC_RESOLUTION 65536

static EL4134_STRUCT ec_el4134[MAX_NB_EL4134];

#define Beckhoff_EL4134 0x00000002, 0x10263052


int config_el4134(  unsigned int id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain1)
{
    if ( id > MAX_NB_EL4134)
        return -1;

    /* Initialisation des paramètres de la structure*/


    if (!(ec_el4134[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL4134)))
        return -1;

    //Output 1 (HR4 gauche)
    ec_el4134[id].offset_output[0]= ecrt_slave_config_reg_pdo_entry(ec_el4134[id].sc,
                                                                 0x7000,
                                                                 0x01,
                                                                 pdomain1,
                                                                 NULL);
    if (ec_el4134[id].offset_output[0] < 0)
        return -1;

    //Output 2 (HR4 droit)
    ec_el4134[id].offset_output[1] = ecrt_slave_config_reg_pdo_entry(ec_el4134[id].sc,
                                                                    0x7010,
                                                                    0x01,
                                                                    pdomain1,
                                                                    NULL);
    if (ec_el4134[id].offset_output[1] < 0)
        return -1;

    if (!(ec_el4134[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL4134)))
        return -1;

    //Output 3 (APA)
    ec_el4134[id].offset_output[2]= ecrt_slave_config_reg_pdo_entry(ec_el4134[id].sc,
                                                                 0x7020,
                                                                 0x01,
                                                                 pdomain1,
                                                                 NULL);
    if (ec_el4134[id].offset_output[2] < 0)
        return -1;


    //Output 4
    ec_el4134[id].offset_output[3] = ecrt_slave_config_reg_pdo_entry(ec_el4134[id].sc,
                                                                    0x7030,
                                                                    0x01,
                                                                    pdomain1,
                                                                    NULL);
    if (ec_el4134[id].offset_output[3] < 0)
        return -1;
    return 0;
}


int write_to_el4134(unsigned int id, uint8_t *pdomain_pd, double *output_value)
{
    int16_t dac_value;
    int i;
    if ( id > MAX_NB_EL4134)
        return -1;

    for (i = 0 ; i < NB_OUTPUT ; i++)
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
        dac_value = (int16_t)(output_value[i]*3276.7);//(double)DAC_RESOLUTION/(MAX_OUTPUT_VALUE-MIN_OUTPUT_VALUE));
       // printf("%d %d\n",i,dac_value);
        EC_WRITE_S16(pdomain_pd + ec_el4134[id].offset_output[i],dac_value);
    }
    return 0;
}
