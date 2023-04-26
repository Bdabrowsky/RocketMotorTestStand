typedef struct{
    float load_cell_range;
    float load_cell_sensitivity;
    float tare_force;
}utils_data_t;

void  UtilsInit(float load_cell_range, float load_cell_sensitivity, float tare_force);
float DataToVoltage(unsigned long data);
float ConvertVoltage(float voltage, int adc_channel);
float UtilsSrv(unsigned long data);