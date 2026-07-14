#include "Hardware/uart/uart.h"

static EncoderFrame s_txFrame;

static uint8_t UART_CalcChecksum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}


void UART2_SendEncoderData_DMA(int32_t left_pulses, int32_t right_pulses)
{
    s_txFrame.head1 = 0xAA;
    s_txFrame.head2 = 0xBB;
    s_txFrame.tail1 = 0xCC;
    s_txFrame.tail2 = 0xDD;

    s_txFrame.left_count  = left_pulses;
    s_txFrame.right_count = right_pulses;

    uint8_t *pData = (uint8_t *)&s_txFrame;
    s_txFrame.checksum = UART_CalcChecksum(pData, 10);

    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)pData);
    DL_DMA_setDestAddr(DMA,DMA_CH1_CHAN_ID,(uint32_t)&UART_32_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, sizeof(EncoderFrame));  //13字节
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}






