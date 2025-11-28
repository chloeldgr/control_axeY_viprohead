#ifndef __EC_EL1008_H__
#define __EC_EL1008_H__

#include "ecrt.h"
#include <stdio.h>

#define MAX_NB_EL1008 20
#define NB_INPUT_EL1008 8

typedef struct{
    int offset_input;
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL1008_STRUCT;
int EL1008_Config(uint8_t id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);
int EL1008_Read(    uint8_t id,
                    uint8_t *pdomain1,
                    uint8_t *reg);

#endif
