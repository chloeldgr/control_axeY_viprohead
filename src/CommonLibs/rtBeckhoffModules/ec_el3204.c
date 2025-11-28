#include "ec_el3204.h"

static EL3204_STRUCT ec_el3204[MAX_NB_EL3204];
static uint32_t reg_RTDSettings[4] = {0x8000,
                                        0x8010,
                                     0x8020,
                                     0x8030};
static uint16_t reg_RTDElement = 0x19;
static uint16_t reg_Presentation = 0x02;
static uint16_t index_channel[NB_CHANNEL_EL3204] = {0x6000,
                                                    0x6010,
                                                    0x6020,
                                                    0x6030};
//#define RTD_ELEMENT_TYPE

#define Beckhoff_EL3204  0x00000002, 0x0c843052



/**
 * Configuration des élèments pour la mesure de température
 */

int el3204_SetRTDElement(       unsigned int module_id,
                                int id_channel,
                                uint16_t value)
{
    int error;
    error = ecrt_slave_config_sdo16(ec_el3204[module_id].sc, reg_RTDSettings[id_channel], reg_RTDElement, value);
    if ( error )
        return -1;
    return 0;
}

/**
 * @fn el3204_SetPresentationSigned
 * @brief Présentation des données sous forme signée
 * @param module_id
 * @param id_channel
 * @return
 */
int el3204_SetPresentationSigned(unsigned int module_id,
                                 int id_channel)
{
    int error;
    error = ecrt_slave_config_sdo8(ec_el3204[module_id].sc, reg_RTDSettings[id_channel], reg_Presentation, 0);
    if ( error )
        return -1;
    return 0;
}

/**
 * @fn el3204_SetPresentationAbsolute
 * @brief Présentaiton des données sous MSB
 * @param module_id
 * @param id_channel
 * @return
 */
int el3204_SetPresentationAbsolute(unsigned int module_id,
                                   int id_channel)
{
    int error;
    error = ecrt_slave_config_sdo8(ec_el3204[module_id].sc, reg_RTDSettings[id_channel], reg_Presentation, 1);
    if ( error )
        return -1;
    return 0;
}
/**
 * @fn el3204_SetPresentationHighResolution
 * @brief Présentation des mesures sous forme haute résolution valuer décimal *1/100 =T°C (1/100 peut être réglé)
 * @param module_id
 * @param id_channel
 * @return
 */
int el3204_SetPresentationHighResolution(unsigned int module_id,
                                         int id_channel)
{
    int error;
    error = ecrt_slave_config_sdo8(ec_el3204[module_id].sc, reg_RTDSettings[id_channel], reg_Presentation, 2);
    if ( error )
        return -1;
    return 0;
}



int config_el3204(  unsigned int id,
                    uint16_t alias,
                    uint16_t position,
                    ec_master_t *pmaster,
                    ec_domain_t *pdomain)
{
    int i_ch; /* channel index*/
 int bit_position;
    if ( id > MAX_NB_EL3204)
        return -1;




    if (!(ec_el3204[id].sc = ecrt_master_slave_config(  pmaster,
                                                        alias,
                                                        position,
                                                        Beckhoff_EL3204)))
        return -1;

    /*SDO Configure */
   /* for ( i_ch= 0 ; i_ch< NB_CHANNEL_EL3204;i_ch++)
    {
        if ( el3204_SetRTDElement(id, i_ch, RTD_ELEMENT_PT1000 ) < 0)
            printf("EL3204 :Echec configuration RTD_ELEMENT\n");
        if ( el3204_SetPresentationHighResolution(id, i_ch) < 0)
            printf("EL3204 : Echec configuration Presentation\n");

    }*/


    for (i_ch = 0; i_ch < NB_CHANNEL_EL3204; i_ch++)
    {
        ec_el3204[id].offset_value[i_ch] = ecrt_slave_config_reg_pdo_entry( ec_el3204[id].sc,
                                                                            index_channel[i_ch],
                                                                            0x11,
                                                                            pdomain,
                                                                            NULL);
        if (ec_el3204[id].offset_value[i_ch]  < 0)
            return -1;

        ec_el3204[id].offset_underrange[i_ch] = ecrt_slave_config_reg_pdo_entry(    ec_el3204[id].sc,
                                                                                    index_channel[i_ch],
                                                                                    0x01,
                                                                                    pdomain,
                                                                                     (unsigned int *)&bit_position);
        if (ec_el3204[id].offset_underrange[i_ch]  < 0)
            return -1;

        ec_el3204[id].offset_overrange[i_ch] = ecrt_slave_config_reg_pdo_entry(     ec_el3204  [id].sc,
                                                                                    index_channel[i_ch],
                                                                                    0x02,
                                                                                    pdomain,
                                                                                    (unsigned int *)&bit_position);
        if (ec_el3204[id].offset_overrange[i_ch]  < 0)
            return -1;

        ec_el3204[id].offset_limit1[i_ch] = ecrt_slave_config_reg_pdo_entry(  ec_el3204[id].sc,
                                                                            index_channel[i_ch],
                                                                            0x03,
                                                                            pdomain,
                                                                            (unsigned int *)&bit_position);
        if (ec_el3204[id].offset_limit1[i_ch]  < 0)
            return -1;

        ec_el3204[id].offset_limit2[i_ch] = ecrt_slave_config_reg_pdo_entry(  ec_el3204[id].sc,
                                                                            index_channel[i_ch],
                                                                            0x05,
                                                                            pdomain,
                                                                            (unsigned int *)&bit_position);
        if (ec_el3204[id].offset_limit2[i_ch]  < 0)
            return -1;

        ec_el3204[id].offset_error[i_ch] = ecrt_slave_config_reg_pdo_entry(  ec_el3204[id].sc,
                                                                            index_channel[i_ch],
                                                                            0x07,
                                                                            pdomain,
                                                                            (unsigned int *)&bit_position);
        if (ec_el3204[id].offset_error[i_ch]  < 0)
            return -1;

        /* Initialisation des variables de la structure associÃ©e au module*/
        ec_el3204[id].error[i_ch]       = 0;
        ec_el3204[id].limit1[i_ch]      = 0;
        ec_el3204[id].limit2[i_ch]      = 0;
        ec_el3204[id].overrange[i_ch]   = 0;
        ec_el3204[id].underrange[i_ch]  = 0;
        ec_el3204[id].value[i_ch]       = 0;

    }


    return 0;
}

/**
 * @fn read_from_el3204
 * @brief lecture de la mesure de température
 * @param id
 * @param id_channel
 * @param pdomain_pd
 * @param value
 * @return
 */
int read_from_el3204(uint8_t id, uint8_t id_channel, uint8_t *pdomain_pd, int16_t *value)
{
    if (id > MAX_NB_EL3204)
        return -1;

    if ( id_channel > NB_CHANNEL_EL3204)
        return -2;

        ec_el3204[id].value[id_channel] = EC_READ_S16(pdomain_pd+ec_el3204[id].offset_value[id_channel]);

    *value = ec_el3204[id].value[id_channel];

}


