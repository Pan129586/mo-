#include "K230.h"
#include <stdio.h>


uint8_t uart1_rxbuff;           // 串口接收缓存
uint8_t send_buf[5];


void K230_Receive_Data(uint8_t com_data)
{
    uint8_t i;
    static uint8_t RxCounter1 = 0;
    static uint16_t RxBuffer1[6] = {0};
    static uint8_t RxState = 0;

    if(com_data != 0xFF)  // 过滤无效信号
    {
        if(RxState == 0)
        {
            if(com_data == 0x2C) 
            {
                RxState = 1;
                RxBuffer1[RxCounter1++] = com_data;
            }
        }
        else if(RxState == 1)
        {
            if(com_data == 0x12) 
            {
                RxState = 2;
                RxBuffer1[RxCounter1++] = com_data;
            }
            else
            {
                RxState = 0;
                RxCounter1 = 0;
            }
        }
        else if(RxState == 2)
        {
            RxBuffer1[RxCounter1++] = com_data;

            if(RxCounter1 >= 6 || com_data == 0x5B)
            {
                RxState = 3;

				
            }
        }
        else if(RxState == 3)
        {
            if(RxBuffer1[RxCounter1-1] == 0x5B)
            {
                RxCounter1 = 0;
                RxState = 0;
            }
            else
            {
                RxState = 0;
                RxCounter1 = 0;
                for(i=0; i<7; i++)
                {
                    RxBuffer1[i] = 0x00;
                }
            }
        }
    }
}

void UART_K230_INST_IRQHandler(void)
{
    // 检查是否触发了接收中断
    switch(DL_UART_Main_getPendingInterrupt(UART_K230_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            uart1_rxbuff = DL_UART_Main_receiveData(UART_K230_INST);
            K230_Receive_Data(uart1_rxbuff);
            break;
            
        default:
            break;
    }
}