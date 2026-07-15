#include "ti_msp_dl_config.h"
#include "Hardware/DL_control/control.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/OLED_Software_I2C/oled_software_i2c.h"
#include "Hardware/OLED_Software_I2C/meau.h"
#include "Hardware/DL_JY61P/JY61P.h"
#include "Hardware/Graysensor/bsp_Graysensor.h"
#include "Hardware/uart/uart.h"

uint32_t last_show_time = 0;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    PID_param_init();
    Encoder_Init();
    reset_turn_count();
    run_data_init();

    NVIC_SetPriority(TIMER_TICK_INST_INT_IRQN, 1);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_TICK_INST);

    DL_TimerA_startCounter(PWMA_INST);
    DL_TimerA_startCounter(PWMB_INST);
    JY61P_DMA_Init();

    while (1)
    {
        key_press();
        // if ((uint32_t)(ui_time - last_show_time) >= 100)
        // {
        //     last_show_time = ui_time;
        //       Send_To_VOFA(LINE_BASE_SPEED,g_fMotorSpeedCmps,LINE_BASE_SPEED,g_fMotor2SpeedCmps);
            //  UART2_SendEncoderData_DMA((int32_t)g_lMotorPulseSigma,(int32_t)g_lMotor2PulseSigma);
        // }
     
        if ((uint32_t)(ui_time - last_show_time) >= 100U)
        {
            last_show_time = ui_time;
            meau_show();
        }

        // if(flag_20ms ==1)
        // {
        //     // flag_20ms = 0;
            
        //     //     Trace_Task20ms();
            

        // }

    }
}
