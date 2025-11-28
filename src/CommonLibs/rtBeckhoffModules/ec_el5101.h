#ifndef __EC_EL5101_H__
#define __EC_EL5101_H__
#include "ecrt.h"
#include <stdio.h>

#define MAX_NB_EL5101 8 // 256
#define EL5101_NO_ERROR                     0
#define EL5101_MAX_NB_ERROR                 -1
#define EL5101_STATUS_INPUT_ERROR           -2
#define EL5101_OVERRESOLUTION_ERROR         -3
#define EL5101_FAILED_REG_PDO_ENTRY         -4

#define EL5101_MASTER_SDO_UPLOAD_ERROR      -5

typedef struct{
    uint32_t value;
    uint8_t input_status;
    uint8_t overflow;
    uint8_t underflow;
    uint8_t cntset_acc;
    uint8_t lat_ext_val;
    uint8_t lat_val;
    int turn;
    int prevoverfl;
    int prevunderfl;
    int offset_input_status;
    int offset_value_in;
    int offset_ctrl;
    int offset_latch;
    int offset_value_out;


    ec_domain_t *pdomain;
    ec_slave_config_t *sc;
    uint8_t *pdomain_pd;
    uint8_t Reset_C_enable;
    uint16_t alias;
    uint16_t position;
    ec_master_t *pmaster;
    uint8_t bInputStatus;
    uint8_t bInputAStatus;
    uint8_t bInputBStatus;
    uint8_t bInputCStatus;
    uint8_t bOpenCircuit;
    uint8_t bCircuitAFailure;
    uint8_t bCircuitBFailure;
    uint8_t bCircuitCFailure;
}EL5101_STRUCT;

int EL5101_Config(      unsigned int    id,
                        uint16_t        alias,
                        uint16_t        position,
                        uint8_t         Enable_C_Reset,
                        ec_master_t     *pmaster,
                        ec_domain_t     *pdomain1);
int EL5101_Init(  uint8_t id,
                      uint8_t *pdomain_pd);
int EL5101_Write( int id,
                     int value);/* uint32_t value);*/
int EL5101_Read(  uint8_t id,
                      int32_t *encoder_value);

#endif
