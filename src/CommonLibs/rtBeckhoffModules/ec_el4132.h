#ifndef __EC_EL4132_H__
#define __EC_EL4132_H__
#include "ecrt.h"
#include <stdio.h>
#include <math.h>

#define NB_EL4132_CHANNEL 2

typedef struct{
    unsigned int bIsReady;

    int offset_output[NB_EL4132_CHANNEL];
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL4132_STRUCT;

int EL4132_Config(unsigned int id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);

int EL4132_Write(unsigned int id, uint8_t *pdomain_pd, double *output_value);


#endif
