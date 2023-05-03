#pragma once

#include<Arduino.h>

typedef struct{
    float load_cell_range;
    float load_cell_sensitivity;
    float tare_force;
    float Vref;
    uint16_t gain;
}utils_data_t;

void  UtilsInit(float load_cell_range, float load_cell_sensitivity, float tare_force , float Vref, uint16_t gain);
float DataToVoltage(unsigned long data);
float ConvertVoltage(float voltage, int adc_channel);
float UtilsSrv(unsigned long data);