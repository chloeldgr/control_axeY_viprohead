#ifndef __EC_EL4104_H__
#define __EC_EL4104_H__
#include "ecrt.h"
#include <stdio.h>
#include <math.h>


#define MAX_NB_EL4104       10
#define NB_EL4104_CHANNEL   4


typedef struct{
    unsigned int bIsReady;

    int offset_output[NB_EL4104_CHANNEL];
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL4104_STRUCT;

int EL4104_Config(unsigned int id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);

int EL4104_Write(unsigned int id,
                 uint8_t *pdomain_pd,
                 double *output_value);


#endif
