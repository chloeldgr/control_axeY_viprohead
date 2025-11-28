#ifndef __EC_EL3164_H__
#define __EC_EL3164_H__
#include "ecrt.h"
#include <stdio.h>
#include "adc.h"
#define MAX_NB_EL3164 16
#define NB_CHANNEL_EL3164 4




typedef struct{
    unsigned int bIsReady;
    ADC_STRUCT  input[NB_CHANNEL_EL3164];
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;

}EL3164_STRUCT;

int EL3164_Config(unsigned int id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);

int EL3164_Read( uint8_t id, uint8_t *pdomain_pd);
int EL3164_GetValue(  uint8_t id, uint8_t id_channel, double *value);


#endif
