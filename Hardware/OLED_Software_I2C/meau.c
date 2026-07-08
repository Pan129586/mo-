#include "Hardware/OLED_Software_I2C/meau.h"



void App_ShowTraceStatus(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Trace", 16);
    OLED_ShowString(0, 2, (uint8_t *)get_runstate(), 16);
    OLED_ShowString(0, 4, (uint8_t *)"N:", 16);
    OLED_ShowNum(20, 4, g_target_circle, 1, 16);
    OLED_ShowString(48, 4, (uint8_t *)"C:", 16);
    OLED_ShowNum(68, 4, g_compt_corner, 2, 16);
    OLED_ShowString(0, 6, (uint8_t *)"Y:", 16);
    OLED_ShowNum(20, 6, (uint32_t)turn_90_count, 2, 16);
}








