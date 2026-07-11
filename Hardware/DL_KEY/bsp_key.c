#include "bsp_key.h"
int iButtonCount;
int iButtonFlag;
volatile uint8_t g_nButton; 
volatile uint32_t ui_time = 0U;

static uint8_t key1_cnt = 0, key2_cnt = 0, key3_cnt = 0,key4_cnt=0;
static uint8_t key1_flag = 0, key2_flag = 0, key3_flag = 0, key4_flag = 0;

int key_3x3_flag=0;
uint8_t KEY=0;

void ButtonScan(void)
{
    // KEY1 
    if(READ_KEY1)  
    {
        if(key1_cnt < 10) key1_cnt++;
        if(key1_cnt == 10) key1_flag = 1;  
    }
    else  
    {
        if(key1_flag == 1)  
        {
            g_nButton = 1; 
        }
        key1_cnt = 0;       
        key1_flag = 0;   
    }

            // KEY2
    if(READ_KEY2)
    {
        if(key2_cnt < 10) key2_cnt++;
        if(key2_cnt == 10) key2_flag = 1;
    }
    else
    {
        if(key2_flag == 1)
        {
            g_nButton = 2;
        }
        key2_cnt = 0;
        key2_flag = 0;
    }

            // KEY3 
    if(READ_KEY3)
    {
        if(key3_cnt < 10) key3_cnt++;
        if(key3_cnt == 10) key3_flag = 1;
    }
    else
    {
        if(key3_flag == 1)
        {
            g_nButton = 3;
        }
        key3_cnt = 0;
        key3_flag = 0;
    }
            //key4
    if(READ_KEY4)
    {
        if(key4_cnt < 10) key4_cnt++;
        if(key4_cnt == 10) key4_flag = 1;  //10ms的消抖时间
    }
    else
    {
        if(key4_flag == 1)
        {
            g_nButton = 4;
        }
        key4_cnt = 0;
        key4_flag = 0;
    }
}

//滴答定时器进行按键消抖
void SysTick_Handler(void)
{
    ui_time ++;
    ButtonScan();
}
