/**
 * Gestion des modules entrées numériques
 **/

#ifndef __EC_EL1144_H__
#define __EC_EL1144_H__

#include "ecrt.h"
#include <stdio.h>

#define MAX_NB_EL1144 20
#define NB_INPUT_EL1144 4

typedef struct{
    int offset_input;
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
}EL1144_STRUCT;
int config_el1144(uint8_t id,
		  uint16_t alias,
		  uint16_t position,
		  ec_master_t *pmaster,
		  ec_domain_t *pdomain1);
int read_di_el1144( uint8_t id,
		    uint8_t *pdomain1,
		    uint8_t *reg);

#endif
