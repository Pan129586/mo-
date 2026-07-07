/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include "Hardware/DL_control/control.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/OLED_Software_I2C/oled_software_i2c.h"

/* OLED Status */
static void App_ShowTraceStatus(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Trace", 16);
    OLED_ShowString(0, 2, (uint8_t *)Trace_GetStateText(), 16);
    OLED_ShowString(0, 4, (uint8_t *)"N:", 16);
    OLED_ShowNum(20, 4, g_traceTargetLaps, 1, 16);
    OLED_ShowString(48, 4, (uint8_t *)"C:", 16);
    OLED_ShowNum(68, 4, g_traceCompletedCorners, 2, 16);
    OLED_ShowString(0, 6, (uint8_t *)"Y:", 16);
    OLED_ShowNum(20, 6, (uint32_t)turn_90_count, 2, 16);
}

int main(void)
{
    /* System Init */
    SYSCFG_DL_init();
    PID_param_init();
    Encoder_Init();
    reset_turn_count();
    Trace_Init();

    NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_TICK_INST);

    OLED_Init();

    /* Main Loop */
    while (1) {
        Trace_HandleButton();
        App_ShowTraceStatus();
        Delay_ms(200);
    }
}
