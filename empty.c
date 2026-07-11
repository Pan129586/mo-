
#include "ti_msp_dl_config.h"
#include "Hardware/DL_control/control.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/OLED_Software_I2C/oled_software_i2c.h"
#include "Hardware/OLED_Software_I2C/meau.h"
#include "Hardware/DL_JY61P/JY61P.h"
#include "Hardware/Graysensor/bsp_Graysensor.h"



uint32_t last_show_time = 0;

static uint8_t App_TakeTraceTick(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t pending;

    __disable_irq();
    pending = (time_20ms_flag != 0U) ? 1U : 0U;
    time_20ms_flag = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return pending;
}


int main(void)
{
    
    SYSCFG_DL_init();
    OLED_Init();
     PID_param_init();
     Encoder_Init();
     reset_turn_count();
     run_data_init();

    //  NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);   //陀螺仪的串口初始化
     NVIC_SetPriority(TIMER_TICK_INST_INT_IRQN, 1U);
     NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);       
     DL_TimerG_startCounter(TIMER_TICK_INST);
     
     DL_TimerA_startCounter(PWMA_INST);
     DL_TimerA_startCounter(PWMB_INST);

     JY61P_DMA_Init();


    while (1) {
        JY61P_Poll();

        if (App_TakeTraceTick() != 0U)
        {
            Trace_Task20ms();
        }

        key_press();
        if ((uint32_t)(ui_time - last_show_time) >= 100U)
        {
            last_show_time = ui_time;
            meau_show();
        }
    }
}
