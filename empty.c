
#include "ti_msp_dl_config.h"
#include "Hardware/DL_control/control.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/OLED_Software_I2C/oled_software_i2c.h"
#include "Hardware/OLED_Software_I2C/meau.h"

int main(void)
{
    
    SYSCFG_DL_init();
    PID_param_init();
    Encoder_Init();
    reset_turn_count();
    Trace_Init();

    NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);   //陀螺仪的串口初始化
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);       
    DL_TimerG_startCounter(TIMER_TICK_INST);

    OLED_Init();

    while (1) {
        Trace_HandleButton();
        App_ShowTraceStatus();
        Delay_ms(200);
    }
}
