#include "ti_msp_dl_config.h"
#include "Hardware/DL_control/control.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/OLED_Software_I2C/oled_software_i2c.h"
#include "Hardware/OLED_Software_I2C/meau.h"
#include "Hardware/DL_JY61P/JY61P.h"
#include "Hardware/Graysensor/bsp_Graysensor.h"

uint32_t last_show_time = 0U;





int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    PID_param_init();
    Encoder_Init();
    reset_turn_count();
    run_data_init();

    NVIC_SetPriority(TIMER_TICK_INST_INT_IRQN, 1U);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_TICK_INST);

    DL_TimerA_startCounter(PWMA_INST);
    DL_TimerA_startCounter(PWMB_INST);
    JY61P_DMA_Init();

    while (1)
    {


        key_press();
        if ((uint32_t)(ui_time - last_show_time) >= 100U)
        {
            last_show_time = ui_time;
            meau_show();
        }

        if(time_20ms_flag==1)
        {
            time_20ms_flag = 0;
             if (g_tracePhase == TRACE_PHASE_YAW_TURN)
                {
                    yaw_turn_control();
                }
            
            if (Corner_Rise_Flag == 1)   //检测到直角
                {
                    enter_yaw_turn();
                    yaw_turn_control();
                }
            
            
            
        }

        // MotorOutput(100, 100);
        // OLED_ShowString(0, 0,(uint8_t *)"pyq", 16);
    }
}
