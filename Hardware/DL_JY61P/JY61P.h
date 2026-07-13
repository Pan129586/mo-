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

extern volatile uint32_t uart_rx_test_count;
extern volatile float yaw_real;
extern volatile float total_yaw;
extern volatile float last_yaw;
extern volatile float target_yaw;
extern volatile int turn_90_count;
extern volatile uint16_t jy_dma_remain_debug;
extern volatile uint16_t jy_dma_head_debug;
extern volatile uint32_t jy_frame_53_count;
extern volatile uint32_t jy_valid_yaw_count;
extern volatile uint32_t jy_checksum_error_count;
extern volatile uint8_t jy_last_rx_byte;
extern volatile uint32_t jy_header_55_count;
extern volatile uint32_t jy_frame_51_count;
extern volatile uint32_t jy_frame_52_count;
extern volatile uint32_t jy_invalid_type_count;
extern volatile uint32_t jy_poll_budget_hit_count;
extern volatile uint32_t jy_poll_processed_count;


void JY61P_Poll(void);
uint16_t JY61P_PollBudget(uint16_t max_bytes);
void JY61P_DMA_Init(void);
void JY61P_UpdateTurnCounter(void);
void reset_turn_count(void);
float JY61P_angelturn(float angle);

#endif
