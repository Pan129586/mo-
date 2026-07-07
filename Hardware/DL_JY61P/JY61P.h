#ifndef __JY61P_H
#define __JY61P_H

// #include "usart.h"
// #include "K230.h"
// #include "control.h"
#include <math.h> // 为了使用绝对值函数 fabs()
#include <math.h>
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/Graysensor/bsp_Graysensor.h"
// #include "menu.h"


#define JY_RX_BUF_SIZE 256


extern uint8_t Start_Flag;
extern uint8_t jy_state;

extern uint8_t uart2_rxbuff;
extern float wz_real ;
extern volatile float yaw_real;
extern uint8_t jy_stat;
extern volatile float target_yaw;
	

extern uint8_t jy_rx_buf[];
extern volatile uint8_t poker_flag;

void JY61P_Poll(void);
void reset_turn_count(void);

#endif

