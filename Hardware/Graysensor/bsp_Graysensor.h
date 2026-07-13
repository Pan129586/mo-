#ifndef __BSP_GRAYSENSOR_H
#define __BSP_GRAYSENSOR_H

#include "ti_msp_dl_config.h"
#include "Hardware/DL_delay/Delay.h"
#include <stdint.h>

#define THRESHOLD 2000

extern volatile uint8_t State_Value[8];
extern volatile uint16_t sensor_values[8];
extern volatile float Line_Num;
extern volatile uint8_t Corner_Flag;
extern volatile uint8_t Corner_Rise_Flag;
extern volatile uint8_t Black_Sensor_Count;
extern volatile uint32_t g_gray_scan_count;
extern volatile uint32_t g_gray_adc_timeout_count;
extern volatile uint32_t g_corner_event_count;

void Graysensor_Read_All(void);
void Graysensor_ResetState(void);
void Light_Turn_control(void);

#endif
