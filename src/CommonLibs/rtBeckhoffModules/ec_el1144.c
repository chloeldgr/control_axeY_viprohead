#include "ec_el1144.h"

static EL1144_STRUCT ec_el1144[MAX_NB_EL1144];

#define Beckhoff_EL1144 0x00000002, 0x04783052

int config_el1144(uint8_t       id,
		  uint16_t      alias,
		  uint16_t      position,
		  ec_master_t   *pmaster,
		  ec_domain_t   *pdomain1)
{
    if ( id > MAX_NB_EL1144)
	return -1;

    if ( !(ec_el1144[id].sc = ecrt_master_slave_config(pmaster,
						       alias,
						       position,
						       Beckhoff_EL1144)))
	return -1;
    ec_el1144[id].offset_input = ecrt_slave_config_reg_pdo_entry(  ec_el1144[id].sc,
								    0x6000,
								    0x01,
								    pdomain1,
								    NULL);
    if (ec_el1144[id].offset_input < 0)
	return -1;
    return 0;
}


int read_di_el1144(uint8_t id, uint8_t *pdomain1, uint8_t *reg)
{
    *reg = EC_READ_U8(pdomain1+ec_el1144[id].offset_input);
    return 0;
}
