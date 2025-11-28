#ifndef __EC_EL3102_H__
#define __EC_EL3102_H__
#include "ecrt.h"
#include <stdio.h>
#define MAX_NB_EL3102 64

#define NB_CHANNEL 2
static double max_input_voltage = 10.0;
static double min_input_voltage = -10.0;
static unsigned int adc_resolution = 65536;

static uint16_t index_channel[NB_CHANNEL] = {0x3101,
                                             0x3102};
typedef struct{
    int value;

    uint8_t Toggle;
    uint8_t State;
    uint8_t SyncError;
    uint8_t Error;
    uint8_t Limit2;
    uint8_t Limit21;
    uint8_t Limit1;
    uint8_t Limit11;
    uint8_t Overrange;
    uint8_t Underrange;

    int offset_status;
    int offset_value_in;

}ADC_STRUCT;

typedef struct{
    unsigned int bIsReady;
    ADC_STRUCT  input[NB_CHANNEL];
    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
    
}EL3102_STRUCT;

int config_el3102(unsigned int id,
                  uint16_t alias,
                  uint16_t position,
                  ec_master_t *pmaster,
                  ec_domain_t *pdomain1);
int Get_EL3102_Status(uint8_t id);

int read_el3102(uint8_t id,
                uint8_t *pdomain_pd,
                int id_channel,
                double *value);

#endif
