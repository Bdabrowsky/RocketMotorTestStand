#include "Utils.h"
#include<Arduino.h>

utils_data_t utils_data_d;

void UtilsInit(float load_cell_range, float load_cell_sensitivity, float tare_force , float Vref, uint16_t gain){
    utils_data_d.load_cell_range = load_cell_range;
    utils_data_d.load_cell_sensitivity = load_cell_sensitivity;
    utils_data_d.tare_force = tare_force;
    utils_data_d.Vref = Vref;
    utils_data_d.gain = gain;
}

float DataToVoltage(unsigned long data) {
  bool polarity = 0;
  float voltage = 0.0;

  if(polarity == 0){
      voltage = (((double)data / (float)(1ul << 23)) - (float)1) * ((utils_data_d.Vref/2.0) / (float)utils_data_d.gain);
      return voltage; 
  }
  else if(polarity == 1){
      voltage = ((double)data / (float)(1ul << 24) / (float)utils_data_d.gain * (utils_data_d.Vref/2.0));
      return voltage;
  }
  return -1;
}

float ConvertVoltage(float voltage, int adc_channel){
  
  float data;
  
  if(adc_channel == 0){
    data = (utils_data_d.load_cell_range * voltage / (5 * utils_data_d.load_cell_sensitivity)) - utils_data_d.tare_force;
    if(data < 0.0){
      return 0;
    }
    return data;
  }

  /*
  else if(adc_channel == 1){
    data = (pressure_range * voltage / (5 * pressure_sensitivity)) - tare_pressure;
    return data;
  }
  */
  return -1;
}

float UtilsSrv(unsigned long data){
    float voltage = DataToVoltage(data);
    return ConvertVoltage(voltage, 0);
}