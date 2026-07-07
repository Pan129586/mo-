#ifndef __JY61P_H
#define __JY61P_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define JY_RX_BUF_SIZE 256

extern uint8_t Start_Flag;
extern uint8_t jy_state;
extern uint8_t uart2_rxbuff;
extern uint8_t jy_rx_buf[JY_RX_BUF_SIZE];
extern volatile uint16_t jy_rx_head;

extern volatile float yaw_real;
extern volatile float total_yaw;
extern volatile float last_yaw;
extern volatile float target_yaw;
extern volatile int turn_90_count;

void JY61P_Poll(void);
void JY61P_ResetYawTrack(void);
void JY61P_UpdateTurnCounter(void);
void reset_turn_count(void);

#endif
