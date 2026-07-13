#include "JY61P.h"

#include <math.h>

#define JY_TURN_COMPLETE_BIAS_DEG (10.0f)  //10°的误差值，要是小车稳的话可以不需要误差值

volatile uint32_t uart_rx_test_count = 0;
volatile float yaw_real = 0.0f;
volatile float total_yaw = 0.0f;
volatile float last_yaw = 0.0f;
volatile float target_yaw = 0.0f;
volatile int turn_90_count = 0;
volatile uint16_t jy_dma_remain_debug = 0;
volatile uint16_t jy_dma_head_debug = 0;
volatile uint32_t jy_frame_53_count = 0;
volatile uint32_t jy_valid_yaw_count = 0;
volatile uint32_t jy_checksum_error_count = 0;
volatile uint8_t jy_last_rx_byte = 0;
volatile uint32_t jy_header_55_count = 0;
volatile uint32_t jy_frame_51_count = 0;
volatile uint32_t jy_frame_52_count = 0;
volatile uint32_t jy_invalid_type_count = 0;
volatile uint32_t jy_poll_budget_hit_count = 0;
volatile uint32_t jy_poll_processed_count = 0;

uint8_t Start_Flag = 0;
uint8_t uart2_rxbuff = 0;
uint8_t jy_buf[11];
uint8_t jy_state = 0;
uint8_t jy_rx_buf[JY_RX_BUF_SIZE];
volatile uint16_t jy_rx_head = 0;
static uint16_t jy_rx_tail = 0;


void JY61P_DMA_Init(void)
{
    jy_rx_head = 0;
    jy_rx_tail = 0;

    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,(uint32_t)&UART_JY61P_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&jy_rx_buf[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, JY_RX_BUF_SIZE);
    DL_UART_Main_enableDMAReceiveEvent(UART_JY61P_INST,DL_UART_MAIN_DMA_INTERRUPT_RX);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}


 float JY61P_angelturn(float angle)
{
    if (angle < -180.0f) 
    {
        angle += 360.0f;
    }
    else if (angle > 180.0f) 
    {
        angle -= 360.0f;
    }

    return angle;
}


static uint8_t JY61P_ParseYaw(void)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < 10; i++) 
    {
        sum += jy_buf[i];
    }
    if (sum != jy_buf[10]) 
    {
        jy_checksum_error_count++;
        return 0U;
    }

    int16_t yaw_raw = ((int16_t)jy_buf[7] << 8) | jy_buf[6];
    yaw_real = (float)yaw_raw / 32768.0f * 180.0f;
    return 1U;
}


void reset_turn_count(void)
{
    total_yaw = 0.0f;
    turn_90_count = 0;
    target_yaw = 0.0f;
    last_yaw = yaw_real;
}

void JY61P_UpdateTurnCounter(void)
{
    float diff = JY61P_angelturn(yaw_real - last_yaw);

    if (Start_Flag == 1) 
    {
        total_yaw += diff;
        turn_90_count = (int)((fabsf(total_yaw) +JY_TURN_COMPLETE_BIAS_DEG) / 90.0f);  

        if (total_yaw >= 0.0f) 
        {
            target_yaw = (float)turn_90_count * 90.0f;
        } 
        else 
        {
            target_yaw = (float)-turn_90_count * 90.0f; //
        }
    }

    last_yaw = yaw_real;
}



static void JY61P_receiv(uint8_t ch)
{
    static uint8_t cnt = 0;

    switch (jy_state) 
    {
        case 0:
            if (ch == 0x55U) 
            {
                jy_header_55_count++;
                jy_buf[0] = ch;
                jy_state = 1;
            }
            break;
        case 1:
            // 加速度、角速度、角度三种包头
            if (ch == 0x51U || ch == 0x52U || ch == 0x53U) 
            {
                jy_buf[1] = ch;
                if (ch == 0x51U)
                {
                    jy_frame_51_count++;
                } 
                else if (ch == 0x52U) 
                {
                    jy_frame_52_count++;
                }
                cnt = 0;
                jy_state = 2;
            } 
            else 
            {
                jy_invalid_type_count++;
                if (ch == 0x55U) 
                {
                    jy_header_55_count++;
                    jy_buf[0] = ch;
                    jy_state = 1;
                    break;
                }
                jy_state = 0;
            }
            break;
        case 2:
            jy_buf[2 + cnt] = ch;
            cnt++;
            // 凑齐一帧的 11 个字节了
            if (cnt >= 9) 
            {
                cnt = 0;
                jy_state = 0;
                
                if (jy_buf[1] == 0x53U) 
                {
                    jy_frame_53_count++;
                    if (JY61P_ParseYaw() != 0U) 
                    {
                        jy_valid_yaw_count++;
                        JY61P_UpdateTurnCounter(); // 更新转角累计
                    }
                }
            }
            break;
        default:
            cnt = 0;
            jy_state = 0;
            break;
    }
}

//串口接收的环形数组
uint16_t JY61P_PollBudget(uint16_t max_bytes)
{
    uint16_t dma_remain = DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
    uint16_t current_head;
    uint16_t processed = 0U;

    if (max_bytes == 0U)
    {
        return 0U;
    }

    jy_dma_remain_debug = dma_remain;

    if (dma_remain > JY_RX_BUF_SIZE) 
    {
        return 0U;
    }

    current_head = (uint16_t)((JY_RX_BUF_SIZE - dma_remain) % JY_RX_BUF_SIZE);
    jy_rx_head = current_head;

    jy_dma_head_debug = current_head;

    while ((jy_rx_tail != current_head) && (processed < max_bytes))
    {
        uint8_t ch = jy_rx_buf[jy_rx_tail];
        jy_rx_tail = (uint16_t)((jy_rx_tail + 1U) % JY_RX_BUF_SIZE);
        jy_last_rx_byte = ch;
        uart_rx_test_count++;
        JY61P_receiv(ch);
        processed++;
    }

    jy_poll_processed_count += processed;
    if (jy_rx_tail != current_head)
    {
        jy_poll_budget_hit_count++;
    }

    return processed;
}

void JY61P_Poll(void)
{
    (void)JY61P_PollBudget(JY_RX_BUF_SIZE);
}

void UART_JY61P_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_JY61P_INST)) 
    {
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            // uart_rx_test_count++;
            break;
        default:
            break;
            
    }
}
