#include "Hardware/OLED_Software_I2C/meau.h"

uint8_t meau_flag=1;




void key_press(void)
{
    uint8_t key=g_nButton;
    switch (key)
    {
        case KEY1_PRES:
            meau_flag ++;
            if(meau_flag>2)
            {
                meau_flag =1;
            }
            OLED_Clear();
        break;
        case KEY2_PRES:
            if (g_traceState != RUN_STATE_RUNNING)
            {
                if (g_target_circle < maix_circle)
                {
                    g_target_circle++;
                    
                }
                else
                {
                    g_target_circle=1;
                }
                g_traceState = RUN_STATE_READY;
            }
            break;
        
        case KEY3_PRES:
             if (g_traceState != RUN_STATE_RUNNING) 
            {
                run_start();
            }
            break;
        case KEY4_PRES:
            if (g_traceState == RUN_STATE_RUNNING) 
            {
                run_stop(RUN_STATE_EMERGENCY_STOP);
            }
            else 
            {
                 run_data_init();
            }
        break;
        default:
            break;

    }
}



void run_state_show(void)
{
    OLED_ShowString(0, 2,(uint8_t *)get_runstate(),16);
    OLED_ShowString(40, 2,(uint8_t *)"             ",16);

    OLED_ShowString(0, 4, (uint8_t *)"ciecle:", 16);
    OLED_ShowNum(20, 4, g_target_circle, 1, 16);

    OLED_ShowString(48, 4, (uint8_t *)"corner_compt:", 16);
    OLED_ShowNum(68, 4, g_compt_corner, 2, 16);
    OLED_ShowString(92, 4, (uint8_t *)"            ", 16);
}

void YY61P_state_show(void)
{
    OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
    OLED_ShowNum(40, 2, (int)yaw_real, 4, 16); 
    OLED_ShowString(72, 2, (uint8_t *)"    ", 16); 

    OLED_ShowString(0, 6, (uint8_t *)"T90_count:", 16);
    OLED_ShowNum(40, 6, turn_90_count, 2, 16);
    OLED_ShowString(72, 6, (uint8_t *)"    ", 16);

}


void meau_show(void)
{
    if(meau_flag==1)
    {
        YY61P_state_show();
    }
    else if (meau_flag==2) 
    {
        run_state_show();
    }

}
   







