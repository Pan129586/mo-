#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

#include "ti_msp_dl_config.h"
#include "Hardware/DL_delay/Delay.h"

void Buzzer_500ms();


#define Buzzer_on  DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BEEP_PIN)
#define Buzzer_off DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BEEP_PIN)

#endif

