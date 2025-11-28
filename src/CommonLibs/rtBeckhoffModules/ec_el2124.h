#ifndef __EC_EL2124_H__
#define __EC_EL2124_H__
#include "ecrt.h"
#include <stdio.h>

#define MAX_NB_EL2124 20

typedef struct{
    unsigned int bIsReady;
    int offset_output;
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL2124_STRUCT;

int EL2124_Config(uint8_t id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);
int EL2124_Write(uint8_t id,
                    uint8_t *pdomain_pd,
                    uint8_t reg);


#endif
