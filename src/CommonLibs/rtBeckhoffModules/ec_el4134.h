#ifndef __EC_EL4134_H__
#define __EC_EL4134_H__
#include "ecrt.h"
#include <stdio.h>
#include <math.h>


typedef struct{
    unsigned int bIsReady;

    int offset_output[4];
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL4134_STRUCT;

int config_el4134(unsigned int id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);

int write_to_el4134(unsigned int id, uint8_t *pdomain_pd, double *output_value);


#endif
