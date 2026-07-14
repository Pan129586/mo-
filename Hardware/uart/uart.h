#ifndef __UART_H__
#define __UART_H__


#include "ti_msp_dl_config.h" 
#include <stdint.h>
#include "Hardware/DL_Emcoder/Encoder.h"


#pragma pack(push, 1)
typedef struct
{
    uint8_t head1;       /* 0xAA */
    uint8_t head2;       /* 0xBB */
    int32_t left_count;  /* 左轮累计脉冲 */
    int32_t right_count; /* 右轮累计脉冲 */
    uint8_t checksum;    /* 数据校验 */
    uint8_t tail1;       /* 0xCC */
    uint8_t tail2;       /* 0xDD */
} EncoderFrame;
#pragma pack(pop)



void UART2_SendEncoderData_DMA(int32_t left_pulses, int32_t right_pulses);











#endif