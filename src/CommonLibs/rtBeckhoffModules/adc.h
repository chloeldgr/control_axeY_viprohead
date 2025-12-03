#ifndef __ADC_H__
#define __ADC_H__

typedef struct{
    int value;
    __uint8_t Toggle;
    __uint8_t State;
    __uint8_t SyncError;
    __uint8_t Error;
    __uint8_t Limit2;
    __uint8_t Limit21;
    __uint8_t Limit1;
    __uint8_t Limit11;
    __uint8_t Overrange;
    __uint8_t Underrange;

    int offset_status;
    int offset_value_in;

    double max_input_voltage;
    double min_input_voltage;
    double adc_resolution;

    
}ADC_STRUCT;

#endif