#ifndef __EC_EL2212_H__
#define __EC_EL2212_H__
#include "ecrt.h"
#include <stdio.h>

#define MAX_NB_EL2212 20

typedef struct{
    unsigned int bIsReady;
    int offset_output[2];
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL2212_STRUCT;

int EL2212_Config(uint8_t id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);
int EL2212_Write( uint8_t id,
                  uint8_t *pdomain_pd,
                  uint8_t reg);


#endif
