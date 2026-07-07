#ifndef __IIC_H
#define __IIC_H



#include "ti_msp_dl_config.h"
#include "Hardware/DL_delay/Delay.h"
#include <stdint.h>

#define Asr_Addr          0x34  
#define ASR_RESULT_ADDR   0x64
#define ASR_SPEAK_ADDR    0x6E
#define ASR_CMDMAND       0x00
#define ASR_ANNOUNCER     0xFF

// SCL 时钟线控制宏
#define IIC_SCL_H()   DL_GPIO_setPins(GPIO_VOICE_PORT, GPIO_VOICE_SCL_PIN)
#define IIC_SCL_L()   DL_GPIO_clearPins(GPIO_VOICE_PORT, GPIO_VOICE_SCL_PIN)

// SDA 数据线控制宏
#define IIC_SDA_H()   DL_GPIO_setPins(GPIO_VOICE_PORT, GPIO_VOICE_SDA_PIN)
#define IIC_SDA_L()   DL_GPIO_clearPins(GPIO_VOICE_PORT, GPIO_VOICE_SDA_PIN)

// 读取 SDA 线电平状态 (注意：TI库返回的是掩码，所以加上 != 0 转成布尔值 1 或 0)
#define READ_SDA      ((DL_GPIO_readPins(GPIO_VOICE_PORT, GPIO_VOICE_SDA_PIN) != 0) ? 1 : 0)

// 函数声明
void iic_delay_us(uint32_t us);
void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void IIC_Init(void);
void Voice_IIC_Start(void);
void Voice_IIC_Stop(void);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t Voice_IIC_Wait_Ack(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);

void CI1302_Speak(uint8_t func_type, uint8_t id);
void Asr_Speak(uint8_t cmd ,uint8_t idNum);
int Asr_Result(void);
#endif
