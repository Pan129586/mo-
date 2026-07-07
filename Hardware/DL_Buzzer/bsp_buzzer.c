#include "bsp_buzzer.h"


void Buzzer_500ms(void)
{
    Buzzer_on;
    Delay_ms(500); 
    Buzzer_off;
}