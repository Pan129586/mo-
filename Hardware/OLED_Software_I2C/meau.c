#include "Hardware/OLED_Software_I2C/meau.h"

uint8_t meau_flag=1;
char str_buf[16];
char str_buf1[16];
char str_buf2[16];


void key_press(void)
{
    uint8_t key=g_nButton;
    if (key == 0)
    {
        return;
    }
    g_nButton = 0;
    
    switch (key)
    {
        case KEY1_PRES:
            meau_flag ++;
            if(meau_flag>3)
            {
                meau_flag =1;
            }
            OLED_Clear();
            meau_show();
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
   OLED_ShowString(0, 0, (uint8_t *)get_runstate(), 16);

    OLED_ShowString(0, 2, (uint8_t *)"circle:", 16);
    OLED_ShowNum(56, 2, g_target_circle, 1, 16);
    OLED_ShowString(0, 4, (uint8_t *)"corner_get:", 16);
    OLED_ShowNum(88, 4, g_compt_corner, 2, 16);


}

void YY61P_state_show(void)
{
    sprintf(str_buf, "Yaw: %.2f", yaw_real); 
    OLED_ShowString(0, 0, (uint8_t *)str_buf, 16);

    sprintf(str_buf2, "Flag: %d", Corner_Flag);
     OLED_ShowString(0, 2, (uint8_t *)str_buf2, 16);

    sprintf(str_buf1, "total_yaw: %.2f", total_yaw);
     OLED_ShowString(0, 6, (uint8_t *)str_buf1, 16);

    // OLED_ShowString(0, 0,  (uint8_t *)"Yaw:", 16);
    // OLED_ShowNum(32, 0, (float)yaw_real, 4, 16); 
    // OLED_ShowString(0, 0,  (uint8_t *)"Yaw:", 16);
    // OLED_ShowNum(32, 0,uart_rx_test_count , 4, 16); 

    // OLED_ShowString(0, 0,  (uint8_t *)"Yaw:", 16);
    // OLED_ShowNum(32, 0,jy_frame_53_count , 4, 16); 

    // sprintf(str_buf, "1: %ld", g_lMotorPulseSigma);
    // OLED_ShowString(0, 2, (uint8_t *)str_buf, 16);

    // sprintf(str_buf, "2: %ld", g_lMotor2PulseSigma);
    // OLED_ShowString(0, 4, (uint8_t *)str_buf, 16);

    // OLED_ShowString(0, 2, (uint8_t *)"T90:", 16);
    // OLED_ShowNum(32, 2, turn_90_count, 2, 16);
    // OLED_ShowString(0, 4, (uint8_t *)"   ", 16);
    // OLED_ShowNum(32, 4, jy_checksum_error_count, 2, 16);
    // OLED_ShowNum(32, 6, jy_header_55_count, 2, 16);



}


void debug_show()
{
    snprintf(str_buf, sizeof(str_buf), "I:%lu",
             (unsigned long)g_trace_isr_count);
    OLED_ShowString(0, 0, (uint8_t *)str_buf, 16);

    snprintf(str_buf2, sizeof(str_buf2), "O:%lu",
             (unsigned long)g_trace_overrun_count);
    OLED_ShowString(0, 2, (uint8_t *)str_buf2, 16);

    snprintf(str_buf1, sizeof(str_buf1), "B:%u F:%u",
             (unsigned int)Black_Sensor_Count,
             (unsigned int)Corner_Flag);
    OLED_ShowString(0, 4, (uint8_t *)str_buf1, 16);

    snprintf(str_buf1, sizeof(str_buf1), "E:%lu C:%u",
             (unsigned long)g_corner_event_count,
             (unsigned int)g_compt_corner);
    OLED_ShowString(0, 6, (uint8_t *)str_buf1, 16);
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
    else if (meau_flag ==3)
    {
        debug_show();

    }

}
