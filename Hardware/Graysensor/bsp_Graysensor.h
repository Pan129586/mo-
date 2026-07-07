#ifndef __BSP_GRAYSENSOR_H
#define __BSP_GRAYSENSOR_H

#include "ti_msp_dl_config.h"
#include "Hardware/DL_delay/Delay.h"
#include <stdint.h>

#define THRESHOLD 2000

extern uint8_t State_Value[8];
extern uint16_t sensor_values[8];
extern float Line_Num;
extern uint8_t Corner_Flag;
extern uint8_t Corner_Rise_Flag;
extern uint8_t Lost_Line_Count;
extern uint8_t Black_Sensor_Count;

void Graysensor_Read_All(void);
void Light_Turn_control(void);

#endif
