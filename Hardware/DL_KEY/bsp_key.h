#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>


#define READ_KEY1  (DL_GPIO_readPins(GPIO_KEY_KEY1_PORT, GPIO_KEY_KEY1_PIN) == 0)
#define READ_KEY2  (DL_GPIO_readPins(GPIO_KEY_KEY2_PORT, GPIO_KEY_KEY2_PIN) == 0)
#define READ_KEY3  (DL_GPIO_readPins(GPIO_KEY_KEY3_PORT, GPIO_KEY_KEY3_PIN) == 0)
#define READ_KEY4  (DL_GPIO_readPins(GPIO_KEY_KEY4_PORT, GPIO_KEY_KEY4_PIN) == 0)


#define KEY1_PRES 1
#define KEY2_PRES 2
#define KEY3_PRES 3
#define KEY4_PRES 4



#define KEY0_DOWN_LEVEL 1

typedef enum
{
  KEY0_UP   = 0,
  KEY0_DOWN = 1,
}KEYState_TypeDef;
 

extern volatile uint8_t ui_time;
extern int iButtonFlag;  
extern volatile uint8_t g_nButton ;
// void KEY_GPIO_Init(void);
uint8_t KEY_Scan(void);

void ButtonScan(void);

KEYState_TypeDef KEY0_StateRead(void);

#endif




