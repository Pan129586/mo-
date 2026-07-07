#ifndef __bsp_Graysensor_H
#define __bsp_Graysensor_H

#include "ti_msp_dl_config.h"
#include  "Hardware/DL_delay/Delay.h"

// #include "bsp_sys.h"
// #include "main.h"
// #include "adc.h"
// #include "K230.h"
// #include "menu.h"
// #include "control.h"
// #include "JY61P.h"
// #include "bsp_delay.h"


#define THRESHOLD 2000  // 黑白分界阈值，需要现场标定

extern uint8_t State_Value[8];
extern uint16_t sensor_values[8];
extern float Line_Num;
extern uint8_t Corner_Flag;


void Graysensor_Read_All(void);
void Light_Turn_control(void);


#endif
