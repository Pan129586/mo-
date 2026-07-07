#ifndef __BSP_LED_H
#define __BSP_LED_H


#include "ti_msp_dl_config.h"

#define BlueSignal_on       DL_GPIO_setPins(GPIO_LED_LED_BLUE_PORT, GPIO_LED_LED_BLUE_PIN)
#define BlueSignal_off      DL_GPIO_clearPins(GPIO_LED_LED_BLUE_PORT, GPIO_LED_LED_BLUE_PIN)
#define BlueSignal_Toggle   DL_GPIO_togglePins(GPIO_LED_LED_BLUE_PORT, GPIO_LED_LED_BLUE_PIN)


#define RedSignal_on        DL_GPIO_setPins(GPIO_LED_LED_RED_PORT, GPIO_LED_LED_RED_PIN)
#define RedSignal_off       DL_GPIO_clearPins(GPIO_LED_LED_RED_PORT, GPIO_LED_LED_RED_PIN)
#define RedSignal_Toggle    DL_GPIO_togglePins(GPIO_LED_LED_RED_PORT, GPIO_LED_LED_RED_PIN)

#endif
