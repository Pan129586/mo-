#ifndef __K230_H
#define __K230_H


#include "ti_msp_dl_config.h" // 替换为 TI 的底层库
#include <stdint.h>           // 使用标准的 C 数据类型

// 全局变量声明 (保持原样)
extern uint8_t str_buff_FPX[64];
extern uint8_t Num, LoR, Finded_flag, FindTask;
extern uint8_t RoomNum, TargetNum, TASK;
extern char TargetRoom;

extern uint8_t FindStartFlag;
extern uint16_t FindTimeCount;
extern uint8_t send_buf[5];

extern uint8_t uart1_rxbuff;


void K230_Receive_Data(uint8_t rx_data);
void SetTargetRoom(void);
void SendDataToK230(void);

#endif
