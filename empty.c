
#include "ti_msp_dl_config.h"
#include "Hardware/DL_control/control.h"
#include "Hardware/DL_Emcoder/Encoder.h"
#include "Hardware/OLED_Software_I2C/oled_software_i2c.h"
#include "Hardware/OLED_Software_I2C/meau.h"
#include "Hardware/DL_JY61P/JY61P.h"



uint32_t last_show_time = 0;


int main(void)
{
    
    SYSCFG_DL_init();
    OLED_Init();
     PID_param_init();
     Encoder_Init();
     reset_turn_count();
     run_data_init();

    //  NVIC_EnableIRQ(UART_JY61P_INST_INT_IRQN);   //陀螺仪的串口初始化
     NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);       
     DL_TimerG_startCounter(TIMER_TICK_INST);
     
     DL_TimerA_startCounter(PWMA_INST);
     DL_TimerA_startCounter(PWMB_INST);

     JY61P_DMA_Init();


    while (1) {
        
        //  key_press();
        //  OLED_ShowString(10,10,(uint8_t *)"pyq",16);
        if ((ui_time - last_show_time) >= 200)
        {
            last_show_time = ui_time;
            meau_show();
        }

        if(time_20ms_flag==1)
        {
            time_20ms_flag =0;
            if(Corner_Flag ==1&& g_tracePhase == TRACE_PHASE_LINE)
            {
                
            }

        }
        
        key_press();
        
    }
}
