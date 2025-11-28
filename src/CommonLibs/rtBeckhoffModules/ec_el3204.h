#ifndef __EC_EL3204_H__
#define __EC_EL3204_H__
#include "ecrt.h"
#include <stdio.h>
#define NB_CHANNEL_EL3204   4
#define MAX_NB_EL3204       32
// Elements RTD supportés
#define RTD_ELEMENT_PT100       0
#define RTD_ELEMENT_Ni100       1
#define RTD_ELEMENT_PT1000      2
#define RTD_ELEMENT_PT500       3
#define RTD_ELEMENT_PT200       4
#define RTD_ELEMENT_Ni1000      5
#define RTD_ELEMENT_Ni1000_1    6
#define RTD_ELEMENT_Ni120       7
#define RTD_ELEMENT_OUTPUT1     8
#define RTD_ELEMENT_OUTPUT2     9






typedef struct{
    uint8_t         underrange[NB_CHANNEL_EL3204];
    uint8_t         overrange[NB_CHANNEL_EL3204];
    uint8_t         limit1[NB_CHANNEL_EL3204];
    uint8_t         limit2[NB_CHANNEL_EL3204];
    uint8_t         error[NB_CHANNEL_EL3204];
    int             value[NB_CHANNEL_EL3204];
    unsigned int    offset_underrange[NB_CHANNEL_EL3204];
    unsigned int    offset_overrange[NB_CHANNEL_EL3204];
    unsigned int    offset_limit1[NB_CHANNEL_EL3204];
    unsigned int    offset_limit2[NB_CHANNEL_EL3204];
    unsigned int    offset_error[NB_CHANNEL_EL3204];
    unsigned int    offset_value[NB_CHANNEL_EL3204];

    ec_domain_t *pdomain;
    ec_slave_config_t *sc;

}EL3204_STRUCT;

int config_el3204(unsigned int  id,
                  uint16_t      alias,
                  uint16_t      position,
                  ec_master_t   *pmaster,
                  ec_domain_t   *pdomain);

int read_from_el3204(uint8_t id,
                     uint8_t id_channel,
                     uint8_t *pdomain_pd,
                     int16_t *value);
#endif
