#include "Utils.h"

utils_data_t utils_data_d;

void UtilsInit(float load_cell_range, float load_cell_sensitivity, float tare_force){
    utils_data_d.load_cell_range = load_cell_range;
    utils_data_d.load_cell_sensitivity = load_cell_sensitivity;
    utils_data_d.tare_force = tare_force;
}

float DataToVoltage(unsigned long data) {
    bool polarity = 1;
    float voltage = 0.0;

    if(polarity == 0){
        voltage = (((float)data / (float)8388608) - (float)1) * ((float)2.5 / (float)128);
        return voltage; 
    }
    else if(polarity == 1){
        voltage = ((double)data / 16777216 / (float)128 * (float)2.5);
        return voltage;
    }
  
}

float ConvertVoltage(float voltage, int adc_channel){
  
  float data;
  
  if(adc_channel == 0){
    data = (utils_data_d.load_cell_range * voltage / (5 * utils_data_d.load_cell_sensitivity)) - utils_data_d.tare_force;
    return data;
  }

  /*
  else if(adc_channel == 1){
    data = (pressure_range * voltage / (5 * pressure_sensitivity)) - tare_pressure;
    return data;
  }
  */
  
}

float UtilsSrv(unsigned long data){
    float voltage = DataToVoltage(data);
    return ConvertVoltage(voltage, 0);
}